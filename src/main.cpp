// VelocityX LFR - DIAGNOSTIC firmware (v3 base + self-test + reasons for every stop/reset)
// Board: Black Pill STM32F411CE, core STM32duino. UNTESTED on hardware: wheels in the air first!
#include <Arduino.h>

// ---------------- Pins ----------------
#define AIN1 PB12
#define AIN2 PB13
#define BIN1 PB14
#define BIN2 PB15
#define PWMA_PIN PB6   // TIM4_CH1
#define PWMB_PIN PB7   // TIM4_CH2
#define STBY_PIN PB5   // driven HIGH. Harmless if STBY is wired to 3V3 instead.
#define LED_PIN PC13   // active LOW
const uint32_t sensorPin[8] = {PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0};
HardwareSerial bt(PA10, PA9);  // RX=PA10 (<-HC-05 TXD), TX=PA9 (->HC-05 RXD)

#define PWM_FREQ 20000
#define PWM_MAX 4095
#define SLEW_PER_MS 6
#define LOST_TIMEOUT_MS 300

// ---------------- State ----------------
float Kp = 0.15f, Kd = 1.8f;
int baseSpeed = 900, maxSpeed = 2200;
bool lineIsLow = true;
bool tel = false;
bool running = false;
HardwareTimer *pwmTim;
uint16_t mn[8], mx[8];
float lastErr = 0;
uint32_t lastSeen = 0, manualUntil = 0;
int curL = 0, curR = 0;
uint32_t rstFlags = 0;
String lastStop = "none";
uint32_t lastStopAt = 0;

void motorSet(uint32_t in1, uint32_t in2, uint32_t ch, int spd) {
  spd = constrain(spd, -PWM_MAX, PWM_MAX);
  if (spd >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); spd = -spd; }
  pwmTim->setCaptureCompare(ch, spd, RESOLUTION_12B_COMPARE_FORMAT);
}
void stopMotors() {
  curL = curR = 0;
  motorSet(AIN1, AIN2, 1, 0);
  motorSet(BIN1, BIN2, 2, 0);
}
int pct2pwm(int pct) { return constrain(pct, -100, 100) * PWM_MAX / 100; }
int slew(int cur, int target) {
  if (target > cur + SLEW_PER_MS) return cur + SLEW_PER_MS;
  if (target < cur - SLEW_PER_MS) return cur - SLEW_PER_MS;
  return target;
}
void drive(int l, int r) {
  l = constrain(l, -maxSpeed, maxSpeed); r = constrain(r, -maxSpeed, maxSpeed);
  curL = slew(curL, l); curR = slew(curR, r);
  motorSet(AIN1, AIN2, 1, curL);
  motorSet(BIN1, BIN2, 2, curR);
}
void stopWhy(const char *why) {
  running = false; manualUntil = 0; stopMotors();
  lastStop = why; lastStopAt = millis();
  bt.print("STOP: "); bt.println(why);
}

void calibrate() {
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
  digitalWrite(LED_PIN, HIGH);
}

float readLine(bool &seen, int &nOn) {
  long num = 0, den = 0; nOn = 0;
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(sensorPin[i]);
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
  bt.print("running="); bt.print(running); bt.print(" base="); bt.print(baseSpeed);
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

void runManual(const char *name, int pctL, int pctR, uint32_t ms) {
  running = false;
  bt.print("RUN "); bt.print(name); bt.print(" L="); bt.print(pctL); bt.print("% R="); bt.print(pctR);
  bt.print("% for "); bt.print(ms); bt.println(" ms");
  motorSet(AIN1, AIN2, 1, pct2pwm(pctL));
  motorSet(BIN1, BIN2, 2, pct2pwm(pctR));
  curL = pct2pwm(pctL); curR = pct2pwm(pctR);
  manualUntil = millis() + ms;
}

void selfTest() {
  running = false;
  bt.println("TEST start: 4 steps, 1 s each, 60%. Watch the wheels.");
  const char *nm[4] = {"left fwd", "left back", "right fwd", "right back"};
  int l[4] = {60, -60, 0, 0}, r[4] = {0, 0, 60, -60};
  for (int i = 0; i < 4; i++) {
    bt.print("TEST "); bt.print(i + 1); bt.print("/4 "); bt.println(nm[i]);
    motorSet(AIN1, AIN2, 1, pct2pwm(l[i]));
    motorSet(BIN1, BIN2, 2, pct2pwm(r[i]));
    delay(1000);
    stopMotors();
    delay(400);
  }
  bt.println("TEST done. If a step did not move: that motor/driver channel is the problem.");
}

void handleCmd(String s) {
  s.trim(); s.toLowerCase();
  if (s == "go") {
    running = true; manualUntil = 0; lastSeen = millis();
    bt.println("GO (sensor mode: stops by itself if line lost 300 ms)");
  }
  else if (s == "stop") { stopWhy("command"); }
  else if (s == "cal") { stopMotors(); running = false; bt.println("CAL 4 s: slide over line"); calibrate(); bt.println("CAL done"); }
  else if (s == "dump") { printDump(); }
  else if (s == "status" || s == "show") { printStatus(); }
  else if (s == "test") { selfTest(); }
  else if (s.startsWith("ml ")) { runManual("left", s.substring(3).toInt(), 0, 3000); }
  else if (s.startsWith("mr ")) { runManual("right", 0, s.substring(3).toInt(), 3000); }
  else if (s.startsWith("fwd ")) { int p = s.substring(4).toInt(); runManual("both", p, p, 3000); }
  else if (s.startsWith("kp ")) { Kp = s.substring(3).toFloat(); bt.print("Kp="); bt.println(Kp); }
  else if (s.startsWith("kd ")) { Kd = s.substring(3).toFloat(); bt.print("Kd="); bt.println(Kd); }
  else if (s.startsWith("base ")) { baseSpeed = s.substring(5).toInt(); bt.print("base="); bt.println(baseSpeed); }
  else if (s.startsWith("max ")) { maxSpeed = s.substring(4).toInt(); bt.print("max="); bt.println(maxSpeed); }
  else if (s.startsWith("pol ")) { lineIsLow = s.substring(4).toInt() != 0; bt.print("lineIsLow="); bt.println(lineIsLow); }
  else if (s.startsWith("tel ")) { tel = s.substring(4).toInt() != 0; bt.print("tel="); bt.println(tel); }
  else bt.println("cmds: test | ml n | mr n | fwd n (n=-100..100, 3 s) | status | dump | cal | go | stop | base n | max n | kp x | kd x | pol 0/1 | tel 0/1");
}

void setup() {
  rstFlags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogReadResolution(12);
  for (int i = 0; i < 8; i++) { mn[i] = 0; mx[i] = 4095; }
  pwmTim = new HardwareTimer(TIM4);
  pwmTim->setPWM(1, PWMA_PIN, PWM_FREQ, 0);
  pwmTim->setPWM(2, PWMB_PIN, PWM_FREQ, 0);
  stopMotors();
  bt.begin(9600);
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PIN, i & 1); delay(100); }
  digitalWrite(LED_PIN, HIGH);
  bt.println("BOOT diag v1 (no auto-calibrate; send cal yourself)");
  printReset();
  bt.println("Send: test   (spins each motor, no sensors needed)");
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
  if (!running) return;
  digitalWrite(LED_PIN, LOW);
  bool seen; int nOn;
  float err = readLine(seen, nOn);
  if (seen) lastSeen = millis();
  else if (millis() - lastSeen > LOST_TIMEOUT_MS) { digitalWrite(LED_PIN, HIGH); stopWhy("line lost (no sensor sees the line)"); return; }
  float corr = Kp * err + Kd * (err - lastErr);
  lastErr = err;
  drive(baseSpeed + (int)corr, baseSpeed - (int)corr);
  if (tel && millis() - tTel > 500) {
    tTel = millis();
    bt.print("run err="); bt.print((int)err); bt.print(" on="); bt.print(nOn);
    bt.print(" L="); bt.print(curL); bt.print(" R="); bt.println(curR);
  }
}
