#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>

// ================= Pins / addresses =================
const uint8_t MPU_ADDR   = 0x69;   // AD0 tied to 5V
const uint8_t LCD_ADDR   = 0x27;
const uint8_t POT_PIN    = A0;
const uint8_t SD_CS      = 10;     // MOSI=11, MISO=12, SCK=13
const uint8_t EVENT_PIN  = 2;      // manual crash button (active low)
const uint8_t RESET_PIN  = 3;      // reset button (active low)
const uint8_t GREEN_PIN  = 4;
const uint8_t RED_PIN    = 5;
const uint8_t BUZZER_PIN = 8;

RTC_DS1307 rtc;                    // Wokwi part is a DS1307; real DS3231 -> RTC_DS3231
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ================= MPU6050 =================
const uint8_t REG_PWR_MGMT1 = 0x6B;
const uint8_t REG_ACCEL_CFG = 0x1C;
const uint8_t REG_ACCEL_OUT = 0x3B;
const float   ACCEL_LSB_PER_G = 4096.0;   // +/-8 g

// ================= Crash detection =================
const uint16_t ACC_THRESH_MG   = 2500;    // impact: above 2.5 g (triggers at once)
const float    TILT_THRESH_DEG = 60.0;    // tilt: above 60 deg ...
const uint8_t  TILT_CONFIRM    = 5;       // ... for 5 samples in a row (0.5 s)

// ================= Timing =================
const unsigned long SAMPLE_MS = 100;      // fixed-rate sampler
const unsigned long LCD_MS    = 250;
const unsigned long LOG_MS    = 1000;     // LOG.CSV: 1 line per second

// ================= State machine =================
// Must stay above the first function: the Arduino build generates function
// prototypes there, and setState(State) needs this type to already exist.
enum State : uint8_t { ST_NORMAL, ST_RECORDING, ST_LOCKED };
State state = ST_NORMAL;

// ================= Simulated GPS =================
// Fixed fake position in micro-degrees (degrees x 1,000,000).
// A real GPS module would replace these two constants later.
const int32_t GPS_LAT_E6 = 48858400L;     //  48.858400
const int32_t GPS_LON_E6 =  2294500L;     //   2.294500
char gpsStr[24];                          // "lat,lon" text, built once in setup()

// ================= Debounced buttons =================
// Defined above the first function so the Arduino build can use the type.
const unsigned long DEBOUNCE_MS = 30;

struct Debounced {
  uint8_t pin;
  bool state;                // stable state: true = pressed
  bool raw;                  // last raw reading
  unsigned long since;       // when the raw reading last changed
};

Debounced btnEvent = { EVENT_PIN, false, false, 0 };
Debounced btnReset = { RESET_PIN, false, false, 0 };

// ================= Circular buffer =================
// 9 bytes per sample x 20 samples = 180 bytes = last 2 s of driving
const uint8_t F_IMPACT = 0x01;
const uint8_t F_TILT   = 0x02;
const uint8_t F_BUTTON = 0x04;

struct __attribute__((packed)) Sample {
  uint32_t t;         // millis() when sampled
  uint16_t accelMg;   // acceleration magnitude in milli-g
  uint8_t  speed;     // km/h
  uint8_t  tilt;      // degrees
  uint8_t  flags;     // F_IMPACT | F_TILT | F_BUTTON
};

const uint8_t BUF_N = 20;
Sample  ring[BUF_N];
uint8_t head  = 0;    // next slot to write
uint8_t count = 0;    // how many valid samples (max BUF_N)

void pushSample(const Sample &s) {
  ring[head] = s;
  head = (head + 1) % BUF_N;
  if (count < BUF_N) count++;
}

void clearBuffer() {
  head = 0;
  count = 0;
}

// ================= State machine data =================
const uint8_t POST_SAMPLES = 20;          // 20 x 100 ms = 2 s after the crash
uint8_t postCount = 0;

const char *trigName = "";
uint32_t crashT = 0;                      // millis() of the triggering sample
char crashStamp[20];                      // "YYYY-MM-DD,HH:MM:SS" of the crash

// ================= Files =================
const char LOG_FILE[]   = "LOG.CSV";
const char CRASH_FILE[] = "CRASH.CSV";
File crashFile;
bool sdOk  = false;
bool mpuOk = true;

// ================= Runtime variables =================
Sample latest = {0, 1000, 0, 0, 0};       // most recent sample, for LCD + LOG.CSV
uint8_t tiltCount = 0;
bool eventLatch = false;

unsigned long nextSample = 0, nextLog = 0, lastLcd = 0;
unsigned long lockedAt = 0;
// Serial 't' test: crash fires 1.5 s after the pot first moves
bool          testArmed = false;
unsigned long testFireAt = 0;             // 0 = pot has not moved yet
uint8_t       testBaseSpeed = 0;
bool buzzOn = false;

const bool BUZZER_ENABLED = true;         // set false to silence the alarm while testing
#define DEBUG_COMMANDS 1                  // 1 = Serial test commands, 0 = production build

int minFreeRam = 32767;                   // lowest free RAM seen (stack depth check)

void (*resetFunc)(void) = 0;              // software reboot (Serial 'r')

// Free RAM = gap between the heap end and the stack pointer (AVR only)
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void noteFreeRam() {
  int f = freeRam();
  if (f < minFreeRam) minFreeRam = f;
}

// Debounce: a change counts only after the reading has stayed the same for
// DEBOUNCE_MS. Returns true once per press (on the press edge, not the release).
bool updateButton(Debounced &b, unsigned long now) {
  bool r = (digitalRead(b.pin) == LOW);    // active low (INPUT_PULLUP)
  if (r != b.raw) {
    b.raw = r;
    b.since = now;
  }
  if ((now - b.since) >= DEBOUNCE_MS && r != b.state) {
    b.state = r;
    return r;
  }
  return false;
}

// Text for the CSV: "48.858400,2.294500"
void fmtCoord(char *out, size_t n, int32_t e6) {
  uint32_t a = (e6 < 0) ? (uint32_t)(-e6) : (uint32_t)e6;
  snprintf_P(out, n, PSTR("%s%lu.%06lu"), e6 < 0 ? "-" : "",
             (unsigned long)(a / 1000000UL), (unsigned long)(a % 1000000UL));
}

void buildGpsStr() {
  char la[13], lo[13];
  fmtCoord(la, sizeof(la), GPS_LAT_E6);
  fmtCoord(lo, sizeof(lo), GPS_LON_E6);
  snprintf_P(gpsStr, sizeof(gpsStr), PSTR("%s,%s"), la, lo);
}

// ================= MPU6050 =================
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_OUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;

  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / ACCEL_LSB_PER_G;
  ay = ry / ACCEL_LSB_PER_G;
  az = rz / ACCEL_LSB_PER_G;
  return true;
}

// Read pot + MPU and fill one compact sample
void takeSample(Sample &s) {
  float ax, ay, az;
  s.t     = millis();
  s.speed = map(analogRead(POT_PIN), 0, 1023, 0, 200);
  s.flags = 0;

  mpuOk = readAccel(ax, ay, az);
  if (!mpuOk) {                            // sensor error: pretend "at rest"
    s.accelMg = 1000;
    s.tilt = 0;
    return;
  }

  float mag  = sqrt(ax * ax + ay * ay + az * az);
  float tilt = atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / PI;

  s.accelMg = (uint16_t)constrain(mag * 1000.0 + 0.5, 0.0, 65535.0);
  s.tilt    = (uint8_t)constrain(tilt + 0.5, 0.0, 255.0);

  if (s.accelMg > ACC_THRESH_MG) s.flags |= F_IMPACT;
  if (tilt > TILT_THRESH_DEG)    s.flags |= F_TILT;
}

// ================= SD helpers =================
bool ensureHeader(const char *name, const __FlashStringHelper *hdr) {
  if (SD.exists(name)) return true;        // never add a second header
  File f = SD.open(name, FILE_WRITE);
  if (!f) return false;
  f.println(hdr);
  f.close();
  return true;
}

bool initSd() {
  if (!SD.begin(SD_CS)) return false;
  bool a = ensureHeader(LOG_FILE,   F("date,time,speed,accel,tilt"));
  bool b = ensureHeader(CRASH_FILE, F("event_date,event_time,t_rel_ms,speed,accel,tilt,phase,trigger,lat,lon"));
  return a && b;
}

void dumpFile(const char *name) {
  File f = SD.open(name, FILE_READ);
  if (!f) {
    Serial.print(F("No "));
    Serial.println(name);
    return;
  }
  Serial.print(F("--- "));
  Serial.print(name);
  Serial.println(F(" ---"));
  while (f.available()) Serial.write(f.read());
  Serial.println(F("--- end ---"));
  f.close();
}

// Normal 1 line/s log (open, append, close = flushed to the card every time)
void logLine() {
  DateTime t = rtc.now();
  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) {
    sdOk = false;
    return;
  }
  char line[48];
  snprintf_P(line, sizeof(line), PSTR("%04d-%02d-%02d,%02d:%02d:%02d,%u,%u.%02u,%u"),
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second(),
           latest.speed, latest.accelMg / 1000, (latest.accelMg % 1000) / 10,
           latest.tilt);
  f.println(line);
  f.close();
}

// One row of CRASH.CSV. t_rel_ms is relative to the triggering sample (0).
void writeCrashRow(const Sample &s, const char *phase, bool doFlush) {
  if (!crashFile) return;
  char line[72];
  snprintf_P(line, sizeof(line), PSTR("%s,%ld,%u,%u.%02u,%u,%s,%s"),
           crashStamp, (long)(s.t - crashT), s.speed,
           s.accelMg / 1000, (s.accelMg % 1000) / 10, s.tilt,
           phase, trigName);
  noteFreeRam();                           // stack is deepest around here
  crashFile.print(line);
  crashFile.print(',');
  crashFile.println(gpsStr);               // simulated GPS: lat,lon
  if (doFlush) crashFile.flush();
}

// Dump the whole circular buffer, oldest first. The newest entry is the trigger.
void writePreCrash() {
  uint8_t start = (head + BUF_N - count) % BUF_N;
  for (uint8_t i = 0; i < count; i++) {
    const Sample &r = ring[(start + i) % BUF_N];
    writeCrashRow(r, (i == count - 1) ? "TRIG" : "PRE", false);
  }
  crashFile.flush();
}

// ================= State transitions =================
void setState(State s) {
  state = s;
  digitalWrite(GREEN_PIN, s == ST_NORMAL ? HIGH : LOW);
  digitalWrite(RED_PIN,   s == ST_NORMAL ? LOW  : HIGH);
  if (s != ST_LOCKED) {
    noTone(BUZZER_PIN);
    buzzOn = false;
  }
}

void startRecording(const Sample &trig, const char *why) {
  trigName = why;
  crashT   = trig.t;
  testArmed = false;                       // a real crash cancels a pending test

  DateTime t = rtc.now();
  snprintf_P(crashStamp, sizeof(crashStamp), PSTR("%04d-%02d-%02d,%02d:%02d:%02d"),
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());

  setState(ST_RECORDING);
  postCount = 0;

  crashFile = SD.open(CRASH_FILE, FILE_WRITE);
  if (crashFile) writePreCrash();
  else sdOk = false;                       // still go on: alarm must work without SD

  Serial.print(F("CRASH: "));
  Serial.println(why);

  nextSample = millis() + SAMPLE_MS;       // don't burst-sample after the slow SD write
}

void finishRecording() {
  if (crashFile) crashFile.close();
  setState(ST_LOCKED);
  lockedAt = millis();
  Serial.println(F("LOCKED: press reset"));
  Serial.print(F("Min free RAM: "));
  Serial.println(minFreeRam);
}

void doReset(unsigned long now) {
  clearBuffer();                           // no stale pre-crash data in the next event
  tiltCount  = 0;
  eventLatch = false;
  setState(ST_NORMAL);
  nextSample = now + SAMPLE_MS;
  nextLog    = now + LOG_MS;
  Serial.println(F("RESET: back to NORMAL"));
}

// ================= Per-sample logic =================
void tickNormal() {
  Sample s;
  takeSample(s);

  // Test mode (Serial 't'): fire a button-style crash 1.5 s after the pot moves
  if (testArmed) {
    if (testFireAt == 0) {
      if (abs((int)s.speed - (int)testBaseSpeed) >= 10) testFireAt = millis() + 1500;
    } else if ((long)(millis() - testFireAt) >= 0) {
      testArmed = false;
      testFireAt = 0;
      eventLatch = true;
    }
  }

  if (eventLatch) {
    s.flags |= F_BUTTON;
    eventLatch = false;
  }
  latest = s;
  pushSample(s);

  tiltCount = (s.flags & F_TILT) ? tiltCount + 1 : 0;

  const char *why = nullptr;
  if (s.flags & F_IMPACT)            why = "IMPACT";
  else if (s.flags & F_BUTTON)       why = "BUTTON";
  else if (tiltCount >= TILT_CONFIRM) why = "TILT";

  if (why) startRecording(s, why);
}

void tickRecording() {
  Sample s;
  takeSample(s);
  latest = s;
  writeCrashRow(s, "POST", true);          // flush every row so a power loss keeps it
  if (++postCount >= POST_SAMPLES) finishRecording();
}

void handleLocked(unsigned long now, bool rstPress) {
  // Alarm: beep 300 ms on / 300 ms off
  bool on = ((now / 300) % 2) == 0;
  if (on != buzzOn) {
    buzzOn = on;
    if (on && BUZZER_ENABLED) tone(BUZZER_PIN, 2000);
    else noTone(BUZZER_PIN);
  }

  // A fresh, debounced press is needed, and presses in the first 0.5 s are ignored
  if (rstPress && now - lockedAt >= 500) doReset(now);
}

// ================= LCD =================
void updateLcd() {
  DateTime now = rtc.now();
  char line[17];

  snprintf_P(line, sizeof(line), PSTR("%02d:%02d:%02d %3dkm/h"),
           now.hour(), now.minute(), now.second(), latest.speed);
  lcd.setCursor(0, 0);
  lcd.print(line);

  if (state == ST_RECORDING) {
    lcd.setCursor(0, 1);
    lcd.print(F("CRASH! recording"));
  } else if (state == ST_LOCKED) {
    lcd.setCursor(0, 1);
    lcd.print(F("CRASH! press RST"));
  } else {
    const char *st = "NORMAL";
    if (!mpuOk)                       st = "MPU ERR";
    else if (latest.flags & F_IMPACT) st = "IMPACT";
    else if (latest.flags & F_TILT)   st = "TILT";
    snprintf_P(line, sizeof(line), PSTR("Status: %-7s%c"), st, sdOk ? 'L' : 'x');
    lcd.setCursor(0, 1);
    lcd.print(line);
  }
}

// ================= Serial commands (testing) =================
// d = LOG.CSV, c = CRASH.CSV, x = clear both files, t = pot test,
// m = free RAM, r = reboot.  Compiled out when DEBUG_COMMANDS is 0.
void handleSerial() {
#if DEBUG_COMMANDS
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') resetFunc();
    if (state == ST_RECORDING) continue;   // don't touch the SD while recording
    if (c == 't') {
      testArmed = true;
      testFireAt = 0;
      testBaseSpeed = latest.speed;
      Serial.println(F("Armed: move the pot, crash fires 1.5 s later"));
    }
    else if (c == 'd') dumpFile(LOG_FILE);
    else if (c == 'c') dumpFile(CRASH_FILE);
    else if (c == 'x') {
      SD.remove(LOG_FILE);
      SD.remove(CRASH_FILE);
      sdOk = initSd();
      Serial.println(F("Files cleared"));
    }
    else if (c == 'm') {
      Serial.print(F("Free RAM now: "));
      Serial.print(freeRam());
      Serial.print(F("  lowest seen: "));
      Serial.println(minFreeRam);
    }
  }
#endif
}

// ================= setup / loop =================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(EVENT_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Power-on self-test: green LED, red LED, one short beep
  digitalWrite(GREEN_PIN, HIGH);
  delay(150);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
  delay(150);
  digitalWrite(RED_PIN, LOW);
  if (BUZZER_ENABLED) {
    tone(BUZZER_PIN, 1500, 100);
    delay(150);
  }

  buildGpsStr();                           // "lat,lon" text for the crash records

  writeReg(REG_PWR_MGMT1, 0x00);
  writeReg(REG_ACCEL_CFG, 0x10);

  lcd.init();
  lcd.backlight();
  lcd.print(F("Black box boot"));

  if (!rtc.begin()) {
    lcd.clear();
    lcd.print(F("RTC not found"));
    while (true) {}
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // sim only: on real hardware set the RTC once, not on every boot

  sdOk = initSd();
  lcd.setCursor(0, 1);
  lcd.print(sdOk ? F("SD OK          ") : F("SD FAIL        "));
  Serial.println(sdOk ? F("SD ready") : F("SD init failed"));
  Serial.print(F("GPS (simulated): "));
  Serial.println(gpsStr);
  Serial.print(F("Free RAM: "));
  Serial.println(freeRam());
#if DEBUG_COMMANDS
  Serial.println(F("Commands: d=LOG c=CRASH x=clear t=pot test m=RAM r=reboot"));
#endif

  delay(800);
  lcd.clear();

  setState(ST_NORMAL);
  nextSample = millis() + SAMPLE_MS;
  nextLog    = millis() + LOG_MS;
}

void loop() {
  unsigned long now = millis();

  handleSerial();

  // Debounced buttons, read every loop so short presses aren't missed.
  // Both must be updated every time to keep their stable state current.
  bool evPress  = updateButton(btnEvent, now);
  bool rstPress = updateButton(btnReset, now);

  // The event press is latched until the next 100 ms sample consumes it
  if (evPress && state == ST_NORMAL) eventLatch = true;

  if (state == ST_LOCKED) handleLocked(now, rstPress);

  noteFreeRam();

  // Fixed 100 ms sampler (no drift: the next deadline is always +100 ms)
  if ((long)(now - nextSample) > 500) nextSample = now;   // resync after a long stall
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_MS;
    if (state == ST_NORMAL)         tickNormal();
    else if (state == ST_RECORDING) tickRecording();
  }

  // Normal logging: 1 line per second, only in NORMAL
  if (state == ST_NORMAL && sdOk) {
    if ((long)(now - nextLog) > 2000) nextLog = now;
    if ((long)(now - nextLog) >= 0) {
      nextLog += LOG_MS;
      logLine();
    }
  }

  if (now - lastLcd >= LCD_MS) {
    lastLcd = now;
    updateLcd();
  }
}
