/*  ESP32-C3 Smart Clock — v3.0
 *
 *  I2C bus (GPIO8=SDA, GPIO9=SCL) shared by:
 *    SH1106 OLED · DS3231 RTC · SHT41 Sensor
 *
 *  NAVIGATION
 *    Encoder turn  → switch screens (Home ↔ Env ↔ Alarm)
 *    PSH (knob)    → jump to Home from anywhere
 *    On Alarm screen:
 *      CON         → enter time-edit mode (hour first)
 *      BAK         → toggle alarm ON / OFF
 *    In edit mode:
 *      Encoder     → change hour / minute
 *      CON         → advance to next field / save
 *      BAK         → cancel / exit edit
 *    While alarm fires:
 *      Any button  → dismiss
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <RTClib.h>
#include "Adafruit_SHT4x.h"
#include <math.h>
#include <DFRobotDFPlayerMini.h>


// ═══════════════════════════════════════════════════════
//  HARDWARE
// ═══════════════════════════════════════════════════════
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
RTC_DS3231     rtc;
Adafruit_SHT4x sht4;
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfplayer;



// ═══════════════════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════════════════
#define ENC_A    0
#define ENC_B    1
#define ENC_BTN  3   // PSH – knob push
#define BTN_OK   2   // CON – confirm
#define BTN_BACK 4   // BAK – back  ← was GPIO4; 4 is now SDA
#define I2C_SDA  8
#define I2C_SCL  9
#define DF_RX 20
#define DF_TX 21

// ═══════════════════════════════════════════════════════
//  ENCODER  (ISR)
// ═══════════════════════════════════════════════════════
volatile int encCount = 0;
int          encSnap  = 0;

void IRAM_ATTR isrEncoder() {
  static uint32_t lastUs    = 0;
  static uint8_t  pulseCnt  = 0;
  static int8_t   pulseDir  = 0;
  static uint8_t  prevState = 0;

  uint32_t t = micros();
  if (t - lastUs < 1500) return;
  lastUs = t;

  uint8_t a = digitalRead(ENC_A), b = digitalRead(ENC_B);
  uint8_t s  = (a << 1) | b;
  uint8_t tr = (prevState << 2) | s;
  prevState  = s;

  int8_t d = 0;
  if (tr==0b0010||tr==0b1011||tr==0b1101||tr==0b0100) d =  1;
  if (tr==0b0001||tr==0b0111||tr==0b1110||tr==0b1000) d = -1;

  if (d) {
    if (d != pulseDir) { pulseCnt = 0; pulseDir = d; }
    if (++pulseCnt >= 2) { encCount += d; pulseCnt = 0; }
  }
}

// ═══════════════════════════════════════════════════════
//  BUTTONS
// ═══════════════════════════════════════════════════════
struct Btn {
  uint8_t pin;
  bool    prev = true, curr = true;
  void    poll() { prev = curr; curr = (bool)digitalRead(pin); }
  bool    fell() { return !curr && prev; }
};
Btn btnPsh{ENC_BTN}, btnCon{BTN_OK}, btnBak{BTN_BACK};

// ═══════════════════════════════════════════════════════
//  APP STATE
// ═══════════════════════════════════════════════════════
enum Screen  : uint8_t { SCR_HOME=0, SCR_ENV, SCR_ALARM, SCR_N };
enum AlmEdit : uint8_t { AE_VIEW=0, AE_HOUR, AE_MIN };

Screen  curScr  = SCR_HOME;
AlmEdit almEdit = AE_VIEW;

uint8_t  almH = 7, almM = 30;
bool     almOn      = false;
bool     almFiring  = false;
bool     almTrigged = false;   // prevents re-fire within same minute
uint32_t almFireMs  = 0;

float    envT = 20.0f, envH = 50.0f;
uint32_t lastSns = 0;
bool     snsOk = false, rtcOk = false;

const char* DOWS[] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
const char* MONS[] = {"JAN","FEB","MAR","APR","MAY","JUN",
                      "JUL","AUG","SEP","OCT","NOV","DEC"};

// ═══════════════════════════════════════════════════════
//  DRAW HELPERS
// ═══════════════════════════════════════════════════════

void fmt2(char* b, uint8_t v) {
  b[0] = '0' + v / 10;
  b[1] = '0' + v % 10;
  b[2] = '\0';
}

// Filled horizontal progress bar with rounded frame
void hBar(int x, int y, int w, int h, float pct) {
  if (pct < 0) pct = 0;
  if (pct > 1) pct = 1;
  u8g2.drawRFrame(x, y, w, h, 1);
  int f = (int)(pct * (w - 2));
  if (f > 0) u8g2.drawBox(x + 1, y + 1, f, h - 2);
}

// Highlighted button box (inverts when lit)
void btnBox(int x, int y, int w, int h, const char* s, bool lit) {
  if (lit) { u8g2.drawRBox(x, y, w, h, 2); u8g2.setDrawColor(0); }
  else        u8g2.drawRFrame(x, y, w, h, 2);
  u8g2.drawStr(x + (w - u8g2.getStrWidth(s)) / 2, y + h - 2, s);
  u8g2.setDrawColor(1);
}

// Three dots showing current screen position
void navDots(int cx, int y) {
  for (int i = 0; i < (int)SCR_N; i++) {
    int dx = cx + (i - 1) * 8;   // dots at cx-8, cx, cx+8
    if (i == (int)curScr) u8g2.drawBox  (dx - 1, y, 5, 3);
    else                   u8g2.drawFrame(dx - 1, y, 5, 3);
  }
}

// Bell icon, ringing = extra vibration lines
void drawBell(int cx, int y, bool ringing) {
  u8g2.drawCircle(cx, y + 5, 5,
    U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawVLine(cx - 5, y + 5, 6);
  u8g2.drawVLine(cx + 5, y + 5, 6);
  u8g2.drawHLine(cx - 5, y + 11, 11);
  u8g2.drawDisc(cx, y + 13, 2, U8G2_DRAW_ALL);
  if (ringing) {
    u8g2.drawLine(cx - 7, y + 2, cx - 5, y + 6);
    u8g2.drawLine(cx + 7, y + 2, cx + 5, y + 6);
  }
}

// ═══════════════════════════════════════════════════════
//  SCR_HOME — Clock face with date and sensor strip
// ═══════════════════════════════════════════════════════
void renderHome(const DateTime& dt, uint32_t now) {

  // Alive blink dot (top-left)
  bool bl = (now / 600) & 1;
  if (bl) u8g2.drawBox(1, 1, 3, 3);
  else     u8g2.drawFrame(1, 1, 3, 3);

  navDots(108, 1);

  // ── Date bar ─────────────────────────────────────────
  u8g2.setFont(u8g2_font_5x7_tf);
  char db[22];
  snprintf(db, sizeof(db), "%s %02d %s %d",
    DOWS[dt.dayOfTheWeek()], dt.day(), MONS[dt.month()-1], dt.year());
  u8g2.drawStr((128 - u8g2.getStrWidth(db)) / 2, 9, db);
  u8g2.drawHLine(0, 11, 128);

  // ── Big HH:MM with blinking colon ────────────────────
  char hh[3], mm[3];
  fmt2(hh, dt.hour());
  fmt2(mm, dt.minute());

  u8g2.setFont(u8g2_font_fub20_tf);
  int hw = u8g2.getStrWidth(hh);
  int cw = u8g2.getStrWidth(":");
  int mw = u8g2.getStrWidth(mm);
  int sx = (128 - (hw + cw + mw)) / 2;

  u8g2.drawStr(sx,         40, hh);
  if ((now / 500) & 1)
    u8g2.drawStr(sx + hw,  40, ":");
  u8g2.drawStr(sx + hw + cw, 40, mm);

  // ── Seconds progress bar ─────────────────────────────
  int sp = (int)((float)dt.second() / 59.0f * 104.0f);
  u8g2.drawHLine(12, 43, 104);          // track
  if (sp > 0) u8g2.drawBox(12, 41, sp, 3); // fill

  u8g2.drawHLine(0, 46, 128);

  // ── Temp / Humidity strip ─────────────────────────────
  u8g2.setFont(u8g2_font_5x7_tf);
  char tb[12], hb[8];
  if (snsOk) {
    snprintf(tb, sizeof(tb), "%.1f'C", envT);
    snprintf(hb, sizeof(hb),  "%.0f%%", envH);
  } else {
    strcpy(tb, "---'C");
    strcpy(hb, "---%");
  }

  u8g2.drawStr(2,  55, "TEMP");
  u8g2.drawStr(2,  63, tb);
  u8g2.drawVLine(63, 47, 16);
  u8g2.drawStr(66, 55, "HUM");
  u8g2.drawStr(66, 63, hb);
}

// ═══════════════════════════════════════════════════════
//  SCR_ENV — Environment gauges + feels-like
// ═══════════════════════════════════════════════════════
void renderEnv(uint32_t now) {

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "ENVIRONMENT");
  u8g2.drawHLine(0, 12, 128);
  navDots(108, 1);

  u8g2.drawVLine(63, 13, 40);   // column divider

  // ── LEFT: Temperature ────────────────────────────────
  float tPct = snsOk ? constrain(envT / 50.0f, 0.0f, 1.0f) : 0.0f;
  char tv[8];
  if (snsOk) snprintf(tv, sizeof(tv), "%.1f", envT);
  else        strcpy(tv, "---");

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(1, 22, "TEMP");
  u8g2.setFont(u8g2_font_fub11_tf);
  u8g2.drawStr(1, 37, tv);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(1, 44, snsOk ? "'C" : "");
  hBar(1, 46, 60, 6, tPct);

  // ── RIGHT: Humidity ──────────────────────────────────
  float hPct = snsOk ? constrain(envH / 100.0f, 0.0f, 1.0f) : 0.0f;
  char hv[6];
  if (snsOk) snprintf(hv, sizeof(hv), "%.0f", envH);
  else        strcpy(hv, "---");

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(66, 22, "HUM");
  u8g2.setFont(u8g2_font_fub11_tf);
  u8g2.drawStr(66, 37, hv);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(66, 44, snsOk ? "%" : "");
  hBar(66, 46, 60, 6, hPct);

  // ── Bottom: Feels-like (Steadman apparent temp) ──────
  u8g2.drawHLine(0, 54, 128);
  char fb[24];
  if (snsOk) {
    // AT = T + 0.33·e − 4.0   (no wind, Steadman 1994)
    float e  = envH / 100.0f * 6.105f * expf(17.27f * envT / (237.7f + envT));
    float fl = envT + 0.33f * e - 4.0f;
    snprintf(fb, sizeof(fb), "Feels like %.1f'C", fl);
  } else {
    strcpy(fb, "No sensor data");
  }
  u8g2.drawStr((128 - u8g2.getStrWidth(fb)) / 2, 62, fb);
}

// ═══════════════════════════════════════════════════════
//  SCR_ALARM — Alarm time setting
// ═══════════════════════════════════════════════════════
void renderAlarm(uint32_t now) {
  bool bl = (now / 500) & 1;

  // ── Title + bell icon ─────────────────────────────────
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(4, 10, "ALARM");
  drawBell(110, 0, almOn && bl);
  u8g2.drawHLine(0, 12, 128);

  // ── Large alarm time ──────────────────────────────────
  char ah[3], am[3];
  fmt2(ah, almH);
  fmt2(am, almM);

  u8g2.setFont(u8g2_font_fub20_tf);
  int ahw = u8g2.getStrWidth(ah);
  int acw = u8g2.getStrWidth(":");
  int amw = u8g2.getStrWidth(am);
  int ax  = (128 - (ahw + acw + amw)) / 2;
  int ay  = 40;

  // Blinking cursor box around the field being edited
  if (bl) {
    if (almEdit == AE_HOUR)
      u8g2.drawRFrame(ax - 3, ay - 21, ahw + 6, 24, 2);
    else if (almEdit == AE_MIN)
      u8g2.drawRFrame(ax + ahw + acw - 3, ay - 21, amw + 6, 24, 2);
  }

  u8g2.drawStr(ax,           ay, ah);
  u8g2.drawStr(ax + ahw,     ay, ":");
  u8g2.drawStr(ax + ahw + acw, ay, am);

  u8g2.drawHLine(0, 43, 128);

  // ── ON/OFF button + context hint ─────────────────────
  u8g2.setFont(u8g2_font_5x7_tf);
  btnBox(2, 46, 28, 9, almOn ? "ON" : "OFF", almOn);

  const char* hint;
  switch (almEdit) {
    case AE_VIEW: hint = "CON:edit  BAK:tog"; break;
    case AE_HOUR: hint = "Enc:Hr  CON:->min"; break;
    default:      hint = "Enc:Min  CON:save"; break;
  }
  u8g2.drawStr(34, 54, hint);
}

// ═══════════════════════════════════════════════════════
//  ALARM FIRING ANIMATION
// ═══════════════════════════════════════════════════════
void renderAlarmFiring(uint32_t now) {
  uint32_t el   = now - almFireMs;
  uint8_t  beat = (el / 300) % 6;
  bool     inv  = (beat < 3);           // invert every 900 ms

  if (inv) { u8g2.drawBox(0, 0, 128, 64); u8g2.setDrawColor(0); }

  // Expanding concentric rings radiating from centre
  uint8_t rOff = (uint8_t)(el / 40 % 18);
  for (int r = rOff; r < 70; r += 18)
    u8g2.drawCircle(64, 32, r, U8G2_DRAW_ALL);

  // "! ALARM !" with 1-px vertical bounce
  u8g2.setFont(u8g2_font_7x14B_tf);
  const char* txt = "! ALARM !";
  int tw  = u8g2.getStrWidth(txt);
  int yb  = 31 + (int)(beat & 1);
  u8g2.drawStr((128 - tw) / 2, yb, txt);

  // Alarm time subtitle
  char tb[6];
  fmt2(tb, almH); tb[2] = ':'; fmt2(tb + 3, almM); tb[5] = '\0';
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr((128 - u8g2.getStrWidth(tb)) / 2, 46, tb);

  // Dismiss instruction
  u8g2.setFont(u8g2_font_5x7_tf);
  const char* dis = "any button to stop";
  u8g2.drawStr((128 - u8g2.getStrWidth(dis)) / 2, 60, dis);

  u8g2.setDrawColor(1);
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.setPins(I2C_SDA, I2C_SCL);

  pinMode(ENC_A,    INPUT_PULLUP);
  pinMode(ENC_B,    INPUT_PULLUP);
  pinMode(ENC_BTN,  INPUT_PULLUP);
  pinMode(BTN_OK,   INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

  u8g2.begin();
  u8g2.setContrast(255);

  // ── Splash screen ──────────────────────────────────
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tf);
    const char* t1 = "ESP32-C3";
    u8g2.drawStr((128 - u8g2.getStrWidth(t1)) / 2, 24, t1);
    u8g2.setFont(u8g2_font_5x7_tf);
    const char* t2 = "Smart Clock  v3.0";
    u8g2.drawStr((128 - u8g2.getStrWidth(t2)) / 2, 40, t2);
    u8g2.drawHLine(14, 28, 100);
    u8g2.drawHLine(14, 44, 100);
  } while (u8g2.nextPage());
  delay(1200);

  // ── DS3231 RTC ──────────────────────────────────────
  if (rtc.begin()) {
    rtcOk = true;
    if (rtc.lostPower()) {
      // Fallback: set to sketch compile time
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("RTC synced to compile-time");
    }
  } else {
    Serial.println("DS3231 not found!");
  }

  // ── SHT41 sensor ────────────────────────────────────
  if (sht4.begin()) {
    snsOk = true;
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
  } else {
    Serial.println("SHT41 not found!");
  }

  // DFPlayer serial
  dfSerial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);

  while (!dfplayer.begin(dfSerial)) {
  Serial.println("DFPlayer not found");
  while (1);
  }

    dfplayer.volume(20); // 0–30

}

// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  // ── Encoder delta ──────────────────────────────────
  noInterrupts(); int cnt = encCount; interrupts();
  int delta = cnt - encSnap;
  encSnap   = cnt;

  // ── Button poll ────────────────────────────────────
  btnPsh.poll(); btnCon.poll(); btnBak.poll();

  // ── Sensor read (every 3 s) ────────────────────────
  if (snsOk && now - lastSns > 3000) {
    lastSns = now;
    sensors_event_t hEv, tEv;
    sht4.getEvent(&hEv, &tEv);   // humidity first, then temp
    envT = tEv.temperature;
    envH = hEv.relative_humidity;
  }

  // ── RTC snapshot ───────────────────────────────────
  DateTime dt = rtcOk ? rtc.now() : DateTime(2025, 4, 17, 12, 0, 0);

  // ── Alarm trigger ──────────────────────────────────
  if (almOn && !almFiring) {
    bool match = (dt.hour() == almH && dt.minute() == almM);
    if (match && !almTrigged) {
      almFiring  = true;
      almFireMs  = now;
      almTrigged = true;
      startAlarm();
    }
    if (!match) almTrigged = false;   // reset so it fires next day
  }

  // ── Input handling ─────────────────────────────────

  if (almFiring) {
    // Any button dismisses the alarm
    if (btnPsh.fell() || btnCon.fell() || btnBak.fell()) {
      almFiring = false;
      curScr    = SCR_ALARM;
      stopAlarm();
    }

  } else if (almEdit == AE_HOUR) {
    if (delta)         almH    = (almH + delta + 24) % 24;
    if (btnCon.fell()) almEdit = AE_MIN;    // advance to minute
    if (btnBak.fell()) almEdit = AE_VIEW;   // cancel

  } else if (almEdit == AE_MIN) {
    if (delta)         almM    = (almM + delta + 60) % 60;
    if (btnCon.fell()) almEdit = AE_VIEW;   // save & exit
    if (btnBak.fell()) almEdit = AE_VIEW;   // cancel

  } else {
    // Normal screen navigation
    if (delta)
      curScr = (Screen)(((int)curScr + delta + SCR_N) % SCR_N);

    if (curScr == SCR_ALARM) {
      if (btnCon.fell()) almEdit = AE_HOUR;   // enter edit
      if (btnBak.fell()) almOn   = !almOn;     // toggle on/off
    }

    if (btnPsh.fell()) curScr = SCR_HOME;    // knob = jump home
  }

  // ── Render ─────────────────────────────────────────
  u8g2.firstPage();
  do {
    if (almFiring) {
      renderAlarmFiring(now);
    } else {
      switch (curScr) {
        case SCR_HOME:  renderHome(dt, now); break;
        case SCR_ENV:   renderEnv(now);      break;
        case SCR_ALARM: renderAlarm(now);    break;
        default: break;
      }
    }
  } while (u8g2.nextPage());

  delay(20);   // ~50 fps
}

// -------------------------
// START ALARM MUSIC
// -------------------------
void startAlarm() {
  Serial.println("Alarm ON");

  dfplayer.loop(1); // plays file 0001.mp3 in loop
}

// -------------------------
//  STOP ALARM MUSIC
// -------------------------
void stopAlarm() {
  Serial.println("Alarm OFF");

  dfplayer.stop(); // stops playback
}