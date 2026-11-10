// VelocityX LFR - v4 (v3 diagnostic base + follow/go modes)
// Board: Black Pill STM32F411CE, core STM32duino. Wheels in the air first!
//
// NEW COMMANDS
//   follow / follow line          line follow at baseSpeed   (needs "cal" first)
//   follow max / follow line max  line follow at maxSpeed
//   follow 1/2 / follow line 1/2  line follow at half of maxSpeed
//   go                            straight, NO sensors, at baseSpeed (until "stop")
//   go max                        straight at maxSpeed
//   go 1/2                        straight at half of maxSpeed
//   (also works: follow 70 = 70% of maxSpeed, go 30 = 30% of maxSpeed, 1/4, 3/4)
#include <Arduino.h>

// ---------------- Pins ----------------
#define L_IN1_PIN PB6
#define L_IN2_PIN PB7
#define R_IN1_PIN PB8
#define R_IN2_PIN PB1
#define STBY_PIN PB5
#define LED_PIN PC13   // active LOW
const uint32_t sensorPin[8] = {PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0};
HardwareSerial bt(PA10, PA9);  // RX=PA10 (<-HC-05 TXD), TX=PA9 (->HC-05 RXD)

#define PWM_FREQ 20000
#define PWM_MAX 4095
#define SLEW_PER_MS 6
#define LOST_TIMEOUT_MS 300
#define MIN_SPAN 120      // sensor with smaller (max-min) after cal is ignored (low-contrast setup)

enum Mode { M_IDLE, M_FOLLOW, M_STRAIGHT, M_MANUAL };

// ---------------- State ----------------
float Kp = 0.15f, Kd = 1.8f;
int baseSpeed = 900, maxSpeed = 2200;
bool lineIsLow = false;   // your dump: black line gives HIGHER value -> false
bool tel = false;
Mode mode = M_IDLE;
int goSpeed = 0, followSpeed = 900, manL = 0, manR = 0;
bool calibrated = false;
bool ok[8] = {true, true, true, true, true, true, true, true};
HardwareTimer *pwmTim;    // TIM4: PB6, PB7, PB8
HardwareTimer *pwmTim3;   // TIM3: PB1
uint16_t mn[8], mx[8];
float lastErr = 0;
uint32_t lastSeen = 0, manualUntil = 0;
int curL = 0, curR = 0;
uint32_t rstFlags = 0;
String lastStop = "none";
uint32_t lastStopAt = 0;

bool invL = false, invR = false;
void motorPair(HardwareTimer *t1, uint32_t ch1, HardwareTimer *t2, uint32_t ch2, int spd, bool inv) {
  spd = constrain(spd, -PWM_MAX, PWM_MAX);
  if (inv) spd = -spd;
  uint32_t a = spd > 0 ? spd : 0;
  uint32_t b = spd < 0 ? -spd : 0;
  t1->setCaptureCompare(ch1, a, RESOLUTION_12B_COMPARE_FORMAT);
  t2->setCaptureCompare(ch2, b, RESOLUTION_12B_COMPARE_FORMAT);
}
void motorL(int spd) { motorPair(pwmTim, 1, pwmTim, 2, spd, invL); }
void motorR(int spd) { motorPair(pwmTim, 3, pwmTim3, 4, spd, invR); }
void stopMotors() { curL = curR = 0; motorL(0); motorR(0); }
int pct2pwm(int pct) { return constrain(pct, -100, 100) * PWM_MAX / 100; }
int slew(int cur, int target) {
  if (target > cur + SLEW_PER_MS) return cur + SLEW_PER_MS;
  if (target < cur - SLEW_PER_MS) return cur - SLEW_PER_MS;
  return target;
}
void stopWhy(const char *why) {
  mode = M_IDLE; manualUntil = 0; stopMotors();
  lastStop = why; lastStopAt = millis();
  digitalWrite(LED_PIN, HIGH);
  bt.print("STOP: "); bt.println(why);
}

// "max" -> maxSpeed, "1/2" or "half" -> maxSpeed/2, "1/4", "3/4", number = % of maxSpeed, empty -> def
int parseSpeed(String a, int def) {
  a.trim();
  if (a.length() == 0) return def;
  if (a == "max") return maxSpeed;
  if (a == "1/2" || a == "half") return maxSpeed / 2;
  if (a == "1/4") return maxSpeed / 4;
  if (a == "3/4") return maxSpeed * 3 / 4;
  int n = constrain(a.toInt(), 0, 100);
  return maxSpeed * n / 100;
}

// average of 4 reads to reduce ADC noise (important while contrast is low)
uint16_t readSensor(int i) {
  uint32_t s = 0;
  for (int k = 0; k < 4; k++) s += analogRead(sensorPin[i]);
  return s / 4;
}

void calibrate() {
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    digitalWrite(LED_PIN, (millis() / 80) & 1);
    for (int i = 0; i < 8; i++) {
      uint16_t v = readSensor(i);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
    }
  }
  digitalWrite(LED_PIN, HIGH);
  int good = 0;
  for (int i = 0; i < 8; i++) {
    int span = (int)mx[i] - (int)mn[i];
    ok[i] = span >= MIN_SPAN;
    if (ok[i]) good++;
    bt.print("S"); bt.print(i); bt.print(" min="); bt.print(mn[i]);
    bt.print(" max="); bt.print(mx[i]); bt.println(ok[i] ? " OK" : " IGNORED (too small change)");
  }
  calibrated = good >= 2;
  if (!calibrated) bt.println("CAL FAILED: sensors barely change. Lower the sensor board (3-8 mm from floor) / turn the sensor pot, then cal again.");
}

float readLine(bool &seen, int &nOn) {
  long num = 0, den = 0; nOn = 0;
  for (int i = 0; i < 8; i++) {
    if (!ok[i]) continue;
    int raw = readSensor(i);
    int span = max(60, (int)mx[i] - (int)mn[i]);
    long v = constrain(((long)raw - mn[i]) * 1000L / span, 0L, 1000L);
    if (lineIsLow) v = 1000 - v;
    if (v < 250) v = 0; else nOn++;
    num += v * i * 1000L; den += v;
  }
  seen = den > 400;
  if (!seen) return lastErr;
  return (float)num / den - 3500.0f;
}

void printReset() {
  bt.print("reset flags:");
  if (rstFlags & RCC_CSR_PORRSTF)  bt.print(" POR(power lost)");
  if (rstFlags & RCC_CSR_BORRSTF)  bt.print(" BOR(brown-out)");
  if (rstFlags & RCC_CSR_PINRSTF)  bt.print(" PIN(NRST pin)");
  if (rstFlags & RCC_CSR_SFTRSTF)  bt.print(" SOFT");
  if (rstFlags & RCC_CSR_IWDGRSTF) bt.print(" IWDG");
  if (rstFlags & RCC_CSR_WWDGRSTF) bt.print(" WWDG");
  bt.println();
}
void printStatus() {
  bt.print("uptime_s="); bt.println(millis() / 1000);
  printReset();
  bt.print("mode="); bt.print((int)mode); bt.print(" (0 idle,1 follow,2 go,3 manual) calibrated="); bt.println(calibrated);
  bt.print("base="); bt.print(baseSpeed);
  bt.print(" max="); bt.print(maxSpeed); bt.print(" lineIsLow="); bt.println(lineIsLow);
  bt.print("kp="); bt.print(Kp); bt.print(" kd="); bt.println(Kd);
  bt.print("last stop: "); bt.print(lastStop);
  bt.print(" ("); bt.print((millis() - lastStopAt) / 1000); bt.println(" s ago)");
  bt.print("curL="); bt.print(curL); bt.print(" curR="); bt.println(curR);
}
void printDump() {
  for (int i = 0; i < 8; i++) { bt.print(analogRead(sensorPin[i])); bt.print(' '); }
  bt.println();
}

// manual test: ramps up smoothly (slew) to avoid battery sag / brown-out reset
void runManual(const char *name, int pctL, int pctR, uint32_t ms) {
  bt.print("RUN "); bt.print(name); bt.print(" L="); bt.print(pctL); bt.print("% R="); bt.print(pctR);
  bt.print("% for "); bt.print(ms); bt.println(" ms");
  manL = pct2pwm(pctL); manR = pct2pwm(pctR);
  mode = M_MANUAL;
  manualUntil = millis() + ms;
}

void selfTest() {
  mode = M_IDLE; stopMotors();
  bt.println("TEST start: 4 steps, 1 s each, 60%. Watch the wheels.");
  const char *nm[4] = {"left fwd", "left back", "right fwd", "right back"};
  int l[4] = {60, -60, 0, 0}, r[4] = {0, 0, 60, -60};
  for (int i = 0; i < 4; i++) {
    bt.print("TEST "); bt.print(i + 1); bt.print("/4 "); bt.println(nm[i]);
    motorL(pct2pwm(l[i]));
    motorR(pct2pwm(r[i]));
    delay(1000);
    stopMotors();
    delay(400);
  }
  bt.println("TEST done. If a step did not move: that motor/driver channel is the problem.");
}

void handleCmd(String s) {
  s.trim(); s.toLowerCase();
  if (s.startsWith("follow")) {
    String a = s.substring(6);
    a.replace("line", "");
    followSpeed = parseSpeed(a, baseSpeed);
    if (!calibrated) { bt.println("Do 'cal' first (slide sensors over line + floor for 4 s)"); return; }
    lastErr = 0; lastSeen = millis(); manualUntil = 0; mode = M_FOLLOW;
    bt.print("FOLLOW speed="); bt.print(followSpeed); bt.println(" (stops if line lost 300 ms)");
  }
  else if (s == "go" || s.startsWith("go ")) {
    goSpeed = parseSpeed(s.substring(2), baseSpeed);
    manualUntil = 0; mode = M_STRAIGHT;
    bt.print("GO straight speed="); bt.print(goSpeed); bt.println(" (no sensors, send 'stop' to stop)");
  }
  else if (s == "stop") { stopWhy("command"); }
  else if (s == "cal") { mode = M_IDLE; stopMotors(); bt.println("CAL 4 s: slide over line and floor"); calibrate(); bt.println("CAL done"); }
  else if (s == "dump") { printDump(); }
  else if (s == "status" || s == "show") { printStatus(); }
  else if (s == "test") { selfTest(); }
  else if (s.startsWith("ml ")) { runManual("left", s.substring(3).toInt(), 0, 3000); }
  else if (s.startsWith("mr ")) { runManual("right", 0, s.substring(3).toInt(), 3000); }
  else if (s.startsWith("fwd ")) { int p = s.substring(4).toInt(); runManual("both", p, p, 3000); }
  else if (s.startsWith("kp ")) { Kp = s.substring(3).toFloat(); bt.print("Kp="); bt.println(Kp); }
  else if (s.startsWith("kd ")) { Kd = s.substring(3).toFloat(); bt.print("Kd="); bt.println(Kd); }
  else if (s.startsWith("base ")) { baseSpeed = s.substring(5).toInt(); bt.print("base="); bt.println(baseSpeed); }
  else if (s.startsWith("max ")) { maxSpeed = constrain(s.substring(4).toInt(), 0, PWM_MAX); bt.print("max="); bt.println(maxSpeed); }
  else if (s.startsWith("invl ")) { invL = s.substring(5).toInt() != 0; bt.print("invL="); bt.println(invL); }
  else if (s.startsWith("invr ")) { invR = s.substring(5).toInt() != 0; bt.print("invR="); bt.println(invR); }
  else if (s.startsWith("pol ")) { lineIsLow = s.substring(4).toInt() != 0; bt.print("lineIsLow="); bt.println(lineIsLow); }
  else if (s.startsWith("tel ")) { tel = s.substring(4).toInt() != 0; bt.print("tel="); bt.println(tel); }
  else bt.println("cmds: follow [line] [max|1/2|%] | go [max|1/2|%] | stop | cal | dump | status | test | fwd n | ml n | mr n | base n | max n | kp x | kd x | pol 0/1 | invl 0/1 | invr 0/1 | tel 0/1");
}

void setup() {
  rstFlags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
  const uint32_t mPins[4] = {L_IN1_PIN, L_IN2_PIN, R_IN1_PIN, R_IN2_PIN};
  for (int i = 0; i < 4; i++) { pinMode(mPins[i], OUTPUT); digitalWrite(mPins[i], LOW); }
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH);
  analogReadResolution(12);
  for (int i = 0; i < 8; i++) { mn[i] = 0; mx[i] = 4095; }
  pwmTim = new HardwareTimer(TIM4);
  pwmTim->setPWM(1, L_IN1_PIN, PWM_FREQ, 0);
  pwmTim->setPWM(2, L_IN2_PIN, PWM_FREQ, 0);
  pwmTim->setPWM(3, R_IN1_PIN, PWM_FREQ, 0);
  pwmTim3 = new HardwareTimer(TIM3);
  pwmTim3->setPWM(4, R_IN2_PIN, PWM_FREQ, 0);
  stopMotors();
  bt.begin(9600);
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PIN, i & 1); delay(100); }
  digitalWrite(LED_PIN, HIGH);
  bt.println("BOOT v4 (follow / go modes)");
  printReset();
  bt.println("Send: cal, then follow | follow max | follow 1/2 | go | go max | go 1/2 | stop");
}

void loop() {
  static uint32_t tNext = 0, tTel = 0;
  static String line;
  while (bt.available()) {
    char c = bt.read();
    if (c == '\n' || c == '\r') { if (line.length()) handleCmd(line); line = ""; }
    else if (line.length() < 32) line += c;
  }
  if (manualUntil && millis() > manualUntil) { stopWhy("manual timeout (3 s)"); }
  if (micros() < tNext) return;
  tNext = micros() + 1000;
  if (mode == M_IDLE) return;

  int tL = 0, tR = 0;
  float err = 0; int nOn = 0;
  if (mode == M_FOLLOW) {
    digitalWrite(LED_PIN, LOW);
    bool seen;
    err = readLine(seen, nOn);
    if (seen) lastSeen = millis();
    else if (millis() - lastSeen > LOST_TIMEOUT_MS) { stopWhy("line lost (no sensor sees the line)"); return; }
    float corr = Kp * err + Kd * (err - lastErr);
    lastErr = err;
    tL = constrain(followSpeed + (int)corr, -maxSpeed, maxSpeed);
    tR = constrain(followSpeed - (int)corr, -maxSpeed, maxSpeed);
  } else if (mode == M_STRAIGHT) {
    tL = tR = goSpeed;
  } else if (mode == M_MANUAL) {
    tL = manL; tR = manR;
  }
  curL = slew(curL, tL); curR = slew(curR, tR);
  motorL(curL); motorR(curR);

  if (tel && mode == M_FOLLOW && millis() - tTel > 500) {
    tTel = millis();
    bt.print("run err="); bt.print((int)err); bt.print(" on="); bt.print(nOn);
    bt.print(" L="); bt.print(curL); bt.print(" R="); bt.println(curR);
  }
}
