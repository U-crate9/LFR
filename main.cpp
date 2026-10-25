// ===== VelocityX LFR: one file, two modes =====
// BLINK_TEST 1 -> only blinks the onboard LED (use this FIRST to prove flashing works)
// BLINK_TEST 0 -> full line-follower firmware
#define BLINK_TEST 1

#if BLINK_TEST
#include <Arduino.h>
// First test: onboard LED (PC13) blinks every 0.5 s if flashing works.
void setup() { pinMode(PC13, OUTPUT); }
void loop()  { digitalWrite(PC13, !digitalRead(PC13)); delay(500); }

#else
// VelocityX LFR v3b - ANALOG 8-ch sensor, Bluetooth (HC-05) tuning, battery guard
// Board: Black Pill STM32F411CE, core: STM32duino.  UNTESTED on hardware: wheels-up first!
#include <Arduino.h>

// ---------------- Pins ----------------
#define AIN1 PB12
#define AIN2 PB13
#define BIN1 PB14
#define BIN2 PB15
#define PWMA_PIN PB6            // TIM4_CH1
#define PWMB_PIN PB7            // TIM4_CH2
#define LED_PIN  PC13           // active LOW
#define STBY_PIN PB5            // TB6612 STBY (driven HIGH; harmless if STBY is wired to 3V3)
#define KEY_PIN  PA0            // onboard KEY button (active LOW)
#define VBAT_PIN PB1            // divider 10k/10k from battery rail (optional)
const uint32_t sensorPin[8] = {PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0};   // D1..D8
HardwareSerial bt(PA10, PA9);   // HC-05: RX=PA10 (<-TXD), TX=PA9 (->RXD)

// ---------------- Fixed settings ----------------
#define PWM_FREQ 20000          // 20 kHz for coreless motors
#define PWM_MAX  4095
#define LINE_IS_LOW 1           // your test: black line = LOWER voltage. Flip if reversed.
#define USE_VBAT 0              // set 1 only after you fit the 10k/10k divider
#define VBAT_CUTOFF 3.30f
#define AUTO_START 0            // 0 = wait for "go" from phone, 1 = start 3 s after calibration
#define LOST_TIMEOUT_MS 300
#define SLEW_PER_MS 6           // motor ramp limit (soft start)

// ---------------- Tunables (change over Bluetooth) ----------------
float Kp = 0.15f, Kd = 1.8f;
int   baseSpeed = 900;          // ~22 %
int   maxSpeed  = 2200;         // per-motor cap (~54 %) for early tests
bool  running = false;
uint32_t startAt = 0;           // motors wait until this time after "go"/KEY

HardwareTimer *pwmTim;
uint16_t mn[8], mx[8];
float lastErr = 0;
uint32_t lastSeen = 0;
int curL = 0, curR = 0;

void motorSet(uint32_t in1, uint32_t in2, uint32_t ch, int spd) {
  spd = constrain(spd, -PWM_MAX, PWM_MAX);
  if (spd >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else          { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); spd = -spd; }
  pwmTim->setCaptureCompare(ch, spd, RESOLUTION_12B_COMPARE_FORMAT);
}
int slew(int cur, int target) {
  if (target > cur + SLEW_PER_MS) return cur + SLEW_PER_MS;
  if (target < cur - SLEW_PER_MS) return cur - SLEW_PER_MS;
  return target;
}
void drive(int l, int r) {
  l = constrain(l, -maxSpeed, maxSpeed); r = constrain(r, -maxSpeed, maxSpeed);
  curL = slew(curL, l); curR = slew(curR, r);
  motorSet(AIN1, AIN2, 1, curL);   // A = left motor
  motorSet(BIN1, BIN2, 2, curR);   // B = right motor
}
void stopMotors() { curL = curR = 0; motorSet(AIN1, AIN2, 1, 0); motorSet(BIN1, BIN2, 2, 0); }

void calibrate() {               // 4 s: slide robot across the line, white and black under every sensor
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    digitalWrite(LED_PIN, (millis() / 80) & 1);
    for (int i = 0; i < 8; i++) {
      uint16_t v = analogRead(sensorPin[i]);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
    }
  }
  digitalWrite(LED_PIN, LOW);
}

// returns error -3500..+3500 (0 = line under the middle). seen=false if no line.
float readLine(bool &seen) {
  long num = 0, den = 0;
  for (int i = 0; i < 8; i++) {
    int raw  = analogRead(sensorPin[i]);
    int span = max(60, (int)mx[i] - (int)mn[i]);
    long v = constrain(((long)raw - mn[i]) * 1000L / span, 0L, 1000L);   // 0 = darkest..1000 = brightest
    if (LINE_IS_LOW) v = 1000 - v;                                       // make the LINE a big number
    if (v < 250) v = 0;                                                  // ignore weak noise
    num += v * i * 1000L; den += v;
  }
  seen = den > 400;
  if (!seen) return lastErr;
  return (float)num / den - 3500.0f;
}

float readVbat() { return analogRead(VBAT_PIN) * 3.3f / 4095.0f * 2.0f; }

void handleCmd(String s) {
  s.trim(); s.toLowerCase();
  if (s == "go")        { running = true;  startAt = millis() + 1500; lastSeen = startAt; bt.println("GO"); }
  else if (s == "stop") { running = false; stopMotors();        bt.println("STOP"); }
  else if (s == "cal")  { stopMotors(); running = false; calibrate(); bt.println("CAL done"); }
  else if (s.startsWith("kp "))   { Kp = s.substring(3).toFloat();        bt.print("Kp="); bt.println(Kp); }
  else if (s.startsWith("kd "))   { Kd = s.substring(3).toFloat();        bt.print("Kd="); bt.println(Kd); }
  else if (s.startsWith("base ")) { baseSpeed = s.substring(5).toInt();   bt.print("base="); bt.println(baseSpeed); }
  else if (s.startsWith("max "))  { maxSpeed = s.substring(4).toInt();    bt.print("max="); bt.println(maxSpeed); }
  else if (s == "dump") {
    for (int i = 0; i < 8; i++) { bt.print(analogRead(sensorPin[i])); bt.print(' '); }
    bt.println();
  }
  else bt.println("cmds: go stop cal dump kp x kd x base n max n");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH);
  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogReadResolution(12);
  pwmTim = new HardwareTimer(TIM4);
  pwmTim->setPWM(1, PWMA_PIN, PWM_FREQ, 0);
  pwmTim->setPWM(2, PWMB_PIN, PWM_FREQ, 0);
  stopMotors();
  bt.begin(9600);
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PIN, i & 1); delay(250); }
  calibrate();
  bt.println("READY: send go / stop / dump / cal / kp / kd / base / max");
  if (AUTO_START) { delay(3000); running = true; startAt = millis() + 1500; lastSeen = startAt; }
}

void loop() {
  static uint32_t tNext = 0, tBat = 0;
  static String line;
  while (bt.available()) {                       // phone commands
    char c = bt.read();
    if (c == '\n' || c == '\r') { if (line.length()) handleCmd(line); line = ""; }
    else if (line.length() < 32) line += c;
  }
  static bool keyPrev = false; static uint32_t keyT = 0;
  bool key = (digitalRead(KEY_PIN) == LOW);      // KEY button toggles run/stop (no phone needed)
  if (key && !keyPrev && millis() - keyT > 300) {
    keyT = millis(); running = !running;
    if (running) { startAt = millis() + 1500; lastSeen = startAt; } else stopMotors();
  }
  keyPrev = key;
  if (micros() < tNext) return;                  // ~1 kHz loop
  tNext = micros() + 1000;

  if (USE_VBAT && millis() - tBat > 100) {
    tBat = millis();
    if (readVbat() < VBAT_CUTOFF) { running = false; stopMotors(); }
  }
  if (!running) { digitalWrite(LED_PIN, HIGH); return; }
  digitalWrite(LED_PIN, LOW);
  if (millis() < startAt) { stopMotors(); return; }   // 1.5 s countdown

  bool seen;
  float err = readLine(seen);
  if (seen) lastSeen = millis();
  else if (millis() - lastSeen > LOST_TIMEOUT_MS) { running = false; stopMotors(); return; }

  float corr = Kp * err + Kd * (err - lastErr);
  lastErr = err;
  // err > 0 : line is to the RIGHT -> turn right -> left wheel faster, right wheel slower
  drive(baseSpeed + (int)corr, baseSpeed - (int)corr);
}

#endif
