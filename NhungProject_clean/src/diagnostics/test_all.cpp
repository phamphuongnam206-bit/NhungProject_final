// =============================================================================
// test_linh_kien.cpp - Test tung linh kien cua xe qua UART (115200 baud)
//
// Go 1 phim (khong can Enter) de chay tung bai test. So lieu in 1 giay / lan.
//   1 SW1                    2 LED PC13
//   3 Encoder (quay tay)     4 MPU6050
//   5 VL53 J8 rieng          6 VL53 J9 rieng
//   7 VL53 J10 rieng         8 VL53 J11 rieng
//   9 4 VL53 cung bat 1 luc
//   a Motor + encoder TUNG banh (tien/lui)          -> NANG BANH XE
//   c Encoder CA 2 BANH cung chay (tien/lui)        -> NANG BANH XE
//   d BAT TAT CA cung luc: 2 motor + 4 VL53 + gyro + encoder + SW1 + LED, 15 s -> NANG BANH XE
//   b Xe chay: tien, lui, quay trai, quay phai       -> DAT XE XUONG SAN, trong 50 cm
//   0 Chay lan luot tat ca      x Huy bai dang chay
//   + / - Doi PWM thu (mac dinh 80)     h Menu     r Bang ket qua
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#ifndef TEST_BAUD
#define TEST_BAUD 115200
#endif

namespace {

// ---------------- So do chan ----------------
constexpr uint32_t LED_PIN = PC13, SW1_PIN = PB1, STBY_PIN = PB0;
constexpr uint32_t IN1[2] = {PB14, PB12}, IN2[2] = {PB15, PB13}, PWM_PIN[2] = {PB9, PB8};  // 0 trai, 1 phai
constexpr int ENCODER_SIGN[2] = {-1, 1};   // giong firmware chinh: tien => so dem duong
constexpr uint32_t XSHUT[4] = {PA2, PA3, PA4, PA5};
constexpr uint8_t VL_ADDR[4] = {0x30, 0x31, 0x32, 0x33};
const char *const VL_NAME[4] = {"J8 (trai ngoai)", "J9 (truoc trai)", "J10 (truoc phai)", "J11 (phai ngoai)"};

TwoWire bus2(PB11, PB10);
VL53L0X tof[4];
bool tofReady[4] = {false, false, false, false};
uint8_t mpuAddr = 0x68;
bool imuOk = false;
int testPwm = 80;
bool aborted = false;

// ---------------- Ket qua ----------------
enum Res : uint8_t { NOT_RUN, PASS, FAIL, CHECK };
constexpr int N_TEST = 14;
// chi so: 1..9 = phim 1..9, 10 = a, 11 = b, 12 = c, 13 = d
const char *const TEST_NAME[N_TEST] = {"", "SW1", "LED", "Encoder quay tay", "MPU6050", "VL53 J8", "VL53 J9",
                                       "VL53 J10", "VL53 J11", "4 VL53 cung luc", "Motor+encoder tung banh",
                                       "Xe chay tien/lui/trai/phai", "Encoder ca 2 banh", "Tat ca linh kien cung luc"};
Res result[N_TEST] = {};
void setResult(int k, Res r) {
  result[k] = r;
  Serial.print(F("==> KET QUA ")); Serial.print(TEST_NAME[k]); Serial.print(F(": "));
  Serial.println(r == PASS ? F("PASS") : r == FAIL ? F("FAIL") : F("XAC NHAN BANG MAT"));
}

// ---------------- Encoder ----------------
uint16_t prevCnt[2];
int32_t enc[2] = {0, 0};
int32_t counterDelta(uint16_t c, uint16_t p) { const uint16_t d = uint16_t(c - p); return d < 32768U ? int32_t(d) : int32_t(d) - 65536; }
bool setupTimer(TIM_TypeDef *t) {
  t->CR1 = t->CR2 = t->DIER = t->CCER = t->PSC = 0;
  t->ARR = 65535;
  t->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
  t->SMCR = 3;
  t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  t->EGR = TIM_EGR_UG; t->CNT = t->SR = 0; t->CR1 = TIM_CR1_CEN;
  return (t->CR1 & 1) && (t->SMCR & 7) == 3 && t->ARR == 65535;
}
bool initEncoders() {
  __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
  HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
  __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
  const uint32_t remap = AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP;
  AFIO->MAPR = (AFIO->MAPR & ~(remap | AFIO_MAPR_SWJ_CFG)) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  pinMode(PA6, INPUT_PULLUP); pinMode(PA7, INPUT_PULLUP);
  pinMode(PA0, INPUT_PULLUP); pinMode(PA1, INPUT_PULLUP);
  const bool ok = setupTimer(TIM3) && setupTimer(TIM2);
  prevCnt[0] = uint16_t(TIM3->CNT); prevCnt[1] = uint16_t(TIM2->CNT);
  return ok;
}
void readEnc() {
  const uint16_t c[2] = {uint16_t(TIM3->CNT), uint16_t(TIM2->CNT)};
  for (int i = 0; i < 2; ++i) { enc[i] += ENCODER_SIGN[i] * counterDelta(c[i], prevCnt[i]); prevCnt[i] = c[i]; }
}

// ---------------- I2C / MPU ----------------
bool readRegs(TwoWire &b, uint8_t a, uint8_t r, uint8_t *d, uint8_t n) {
  b.beginTransmission(a); b.write(r);
  if (b.endTransmission(false) || b.requestFrom(a, n) != n) return false;
  for (int i = 0; i < n; ++i) { const int v = b.read(); if (v < 0) return false; d[i] = uint8_t(v); }
  return true;
}
bool writeReg(TwoWire &b, uint8_t a, uint8_t r, uint8_t v) { b.beginTransmission(a); b.write(r); b.write(v); return b.endTransmission() == 0; }
bool initImu(uint8_t &who) {
  for (mpuAddr = 0x68; mpuAddr <= 0x69; ++mpuAddr)
    if (readRegs(Wire, mpuAddr, 0x75, &who, 1))
      return writeReg(Wire, mpuAddr, 0x6B, 1) && writeReg(Wire, mpuAddr, 0x1A, 3) &&
             writeReg(Wire, mpuAddr, 0x19, 9) && writeReg(Wire, mpuAddr, 0x1B, 0) && writeReg(Wire, mpuAddr, 0x1C, 0);
  mpuAddr = 0x68;
  return false;
}
bool readGyroZ(float &dps) {
  uint8_t b[2];
  if (!readRegs(Wire, mpuAddr, 0x47, b, 2)) return false;
  dps = int16_t((b[0] << 8) | b[1]) / 131.0f;
  return true;
}
bool readAcc(float &g) {
  uint8_t b[6];
  if (!readRegs(Wire, mpuAddr, 0x3B, b, 6)) return false;
  const float x = int16_t((b[0] << 8) | b[1]), y = int16_t((b[2] << 8) | b[3]), z = int16_t((b[4] << 8) | b[5]);
  g = sqrtf(x * x + y * y + z * z) / 16384.0f;
  return true;
}

// ---------------- Motor ----------------
void motorsOff() {
  digitalWrite(STBY_PIN, LOW);
  for (int i = 0; i < 2; ++i) { analogWrite(PWM_PIN[i], 0); digitalWrite(IN1[i], LOW); digitalWrite(IN2[i], LOW); }
}
void motorBrake() { for (int i = 0; i < 2; ++i) { digitalWrite(IN1[i], HIGH); digitalWrite(IN2[i], HIGH); analogWrite(PWM_PIN[i], 0); } }
void motorSet(int i, int pwm) {   // pwm > 0: tien
  if (!pwm) { digitalWrite(IN1[i], LOW); digitalWrite(IN2[i], LOW); analogWrite(PWM_PIN[i], 0); return; }
  const bool in1High = (pwm > 0) == (i == 1);
  digitalWrite(IN1[i], in1High ? HIGH : LOW);
  digitalWrite(IN2[i], in1High ? LOW : HIGH);
  analogWrite(PWM_PIN[i], min(abs(pwm), 160));
  digitalWrite(STBY_PIN, HIGH);
}

// ---------------- SW1 ----------------
bool swRaw = false, swStable = false;
uint32_t swChanged = 0;
uint16_t swPress = 0, swRelease = 0;
void serviceButton() {
  const uint32_t now = millis();
  const bool down = digitalRead(SW1_PIN) == LOW;
  if (down != swRaw) { swRaw = down; swChanged = now; }
  if (now - swChanged >= 30 && swStable != swRaw) { swStable = swRaw; if (swStable) ++swPress; else ++swRelease; }
}

// Cho ms mili giay; phim x = huy (tat motor ngay). Tra ve false neu bi huy.
bool waitMs(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    serviceButton();
    while (Serial.available()) {
      const char c = char(Serial.read());
      if (c == 'x' || c == 'X') { motorsOff(); aborted = true; Serial.println(F("\r\n!!! DA HUY BAI TEST")); return false; }
    }
  }
  return true;
}
void printSigned(int32_t v) { if (v > 0) Serial.print('+'); Serial.print(v); }
void printSignedF(float v, int d) { if (v > 0) Serial.print('+'); Serial.print(v, d); }

// ---------------- VL53L0X ----------------
void allXshutLow() { for (int i = 0; i < 4; ++i) { pinMode(XSHUT[i], OUTPUT); digitalWrite(XSHUT[i], LOW); tofReady[i] = false; } }
bool bootOne(int i) {   // nha XSHUT cam bien i (cac cam bien chua khoi tao phai dang LOW)
  pinMode(XSHUT[i], INPUT);
  delay(15);
  tof[i] = VL53L0X();
  tof[i].setBus(&bus2);
  tof[i].setTimeout(100);
  bus2.beginTransmission(0x29);
  if (bus2.endTransmission() != 0) {
    Serial.print(F("   ")); Serial.print(VL_NAME[i]); Serial.println(F(": KHONG thay dia chi 0x29 -> kiem tra day SDA/SCL/nguon/XSHUT"));
    pinMode(XSHUT[i], OUTPUT); digitalWrite(XSHUT[i], LOW);
    return false;
  }
  if (!tof[i].init()) {
    Serial.print(F("   ")); Serial.print(VL_NAME[i]); Serial.println(F(": thay 0x29 nhung init() loi"));
    pinMode(XSHUT[i], OUTPUT); digitalWrite(XSHUT[i], LOW);
    return false;
  }
  tof[i].setAddress(VL_ADDR[i]);
  tof[i].setMeasurementTimingBudget(33000);
  tof[i].startContinuous();
  tofReady[i] = !tof[i].last_status;
  Serial.print(F("   ")); Serial.print(VL_NAME[i]); Serial.print(F(" -> dia chi 0x")); Serial.print(VL_ADDR[i], HEX);
  Serial.println(tofReady[i] ? F(" OK") : F(" LOI doi dia chi"));
  return tofReady[i];
}
int bootAll() {
  allXshutLow();
  delay(20);
  int n = 0;
  for (int i = 0; i < 4; ++i) n += bootOne(i);
  return n;
}
// 0: co vat (mm), 1: timeout, 2: khong thay vat (ngoai tam)
int readRange(int i, uint16_t &mm) {
  mm = tof[i].readRangeContinuousMillimeters();
  if (tof[i].timeoutOccurred()) return 1;
  if (mm >= 2000) return 2;
  return 0;
}
// Doc KHONG CHAN: -1 loi I2C, 0 chua co mau moi, 1 co mau moi (mm)
int pollRange(int i, uint16_t &mm) {
  const uint8_t st = tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (tof[i].last_status) return -1;
  if (!(st & 0x07)) return 0;
  mm = tof[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  tof[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  return tof[i].last_status ? -1 : 1;
}
void printRange(int st, uint16_t mm) {
  if (st == 1) Serial.print(F("TIMEOUT"));
  else if (st == 2) Serial.print(F("khong thay"));
  else { Serial.print(mm); Serial.print(F("mm")); }
}

// =============================================================================
// CAC BAI TEST
// =============================================================================
void testSW1() {
  Serial.println(F("\r\n[1] TEST SW1 (PB1, active LOW): nhan va nha SW1 it nhat 3 lan trong 10 s"));
  swPress = swRelease = 0;
  for (int s = 1; s <= 10; ++s) {
    if (!waitMs(1000)) return;
    Serial.print(F("  ")); Serial.print(s); Serial.print(F("s: muc=")); Serial.print(swStable ? F("NHAN") : F("nha "));
    Serial.print(F("  so lan nhan=")); Serial.print(swPress); Serial.print(F("  so lan nha=")); Serial.println(swRelease);
  }
  if (swPress >= 3 && swRelease >= 3) setResult(1, PASS);
  else {
    Serial.println(F("  Goi y: neu muc luon 'NHAN' -> chap PB1 xuong GND; neu luon 'nha' -> nut/day hong"));
    setResult(1, FAIL);
  }
}

void testLED() {
  Serial.println(F("\r\n[2] TEST LED PC13: LED sang 1 s, tat 1 s, 5 lan. Nhin LED tren Blue Pill"));
  for (int k = 1; k <= 5; ++k) {
    digitalWrite(LED_PIN, LOW); Serial.print(F("  lan ")); Serial.print(k); Serial.println(F(": SANG"));
    if (!waitMs(1000)) { digitalWrite(LED_PIN, HIGH); return; }
    digitalWrite(LED_PIN, HIGH); Serial.println(F("         TAT"));
    if (!waitMs(1000)) return;
  }
  setResult(2, CHECK);
}

void testEncoderHand() {
  Serial.println(F("\r\n[3] TEST ENCODER BANG TAY (15 s). Motor TAT."));
  Serial.println(F("  Lam lan luot: quay banh TRAI ve phia TRUOC (so L phai TANG +), roi ve phia SAU (L GIAM -),"));
  Serial.println(F("  sau do quay banh PHAI ve phia TRUOC (R TANG +), roi ve phia SAU (R GIAM -)."));
  motorsOff();
  readEnc();
  const int32_t s0[2] = {enc[0], enc[1]};
  bool plus[2] = {false, false}, minus[2] = {false, false};
  int32_t last[2] = {enc[0], enc[1]};
  for (int s = 1; s <= 15; ++s) {
    if (!waitMs(1000)) return;
    readEnc();
    Serial.print(F("  ")); Serial.print(s); Serial.print(F("s: "));
    for (int i = 0; i < 2; ++i) {
      const int32_t d = enc[i] - last[i];
      last[i] = enc[i];
      if (d > 20) plus[i] = true;
      if (d < -20) minus[i] = true;
      Serial.print(i ? F("  PHAI tong=") : F("TRAI tong=")); printSigned(enc[i] - s0[i]);
      Serial.print(F(" (")); printSigned(d); Serial.print(F("/s ")); Serial.print(d > 20 ? F("TIEN") : d < -20 ? F("LUI ") : F("dung")); Serial.print(')');
    }
    Serial.println();
  }
  for (int i = 0; i < 2; ++i) {
    Serial.print(i ? F("  Banh PHAI: ") : F("  Banh TRAI: "));
    if (!plus[i] && !minus[i]) Serial.println(F("KHONG co xung -> kiem tra day encoder A/B, nguon 3.3V/5V encoder"));
    else if (plus[i] && minus[i]) Serial.println(F("co xung ca 2 chieu"));
    else Serial.println(F("chi thay 1 chieu (ban da quay ca 2 chieu chua?)"));
  }
  Serial.println(F("  Neu quay TIEN ma so lai GIAM -> encoder dao dau (doi ENCODER_SIGN hoac dao day A/B)."));
  setResult(3, (plus[0] || minus[0]) && (plus[1] || minus[1]) ? PASS : FAIL);
}

void testMPU() {
  Serial.println(F("\r\n[4] TEST MPU6050 (I2C1 PB6/PB7), 10 s."));
  uint8_t who = 0;
  imuOk = initImu(who);
  if (!imuOk) { Serial.println(F("  Khong thay 0x68/0x69 -> kiem tra SDA PB7, SCL PB6, VCC, GND")); setResult(4, FAIL); return; }
  Serial.print(F("  Dia chi 0x")); Serial.print(mpuAddr, HEX); Serial.print(F(", WHO_AM_I=0x")); Serial.println(who, HEX);
  Serial.println(F("  3 s dau de xe DUNG YEN; sau do XOAY XE bang tay: sang TRAI (nguoc kim dong ho) roi sang PHAI."));
  int okCount = 0; bool accOk = true; float maxPos = 0, maxNeg = 0, biasSum = 0; int biasN = 0;
  for (int s = 1; s <= 10; ++s) {
    float sum = 0; int n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      float z;
      if (readGyroZ(z)) { sum += z; ++n; }
      if (!waitMs(20)) return;
    }
    float g = 0;
    const bool accRead = readAcc(g);
    const float avg = n ? sum / n : 0;
    if (n > 30) ++okCount;
    if (s <= 3) { biasSum += avg; ++biasN; }
    else { maxPos = fmaxf(maxPos, avg); maxNeg = fminf(maxNeg, avg); }
    if (!accRead || g < 0.85f || g > 1.15f) accOk = false;
    Serial.print(F("  ")); Serial.print(s); Serial.print(F("s: gyroZ=")); printSignedF(avg, 1);
    Serial.print(F(" do/s  gia toc=")); Serial.print(g, 2); Serial.print(F(" g  mau=")); Serial.println(n);
  }
  Serial.print(F("  Bias khi dung yen: ")); printSignedF(biasN ? biasSum / biasN : 0, 2); Serial.println(F(" do/s"));
  Serial.print(F("  Xoay trai lon nhat: ")); printSignedF(maxPos, 1); Serial.print(F("  xoay phai lon nhat: ")); printSignedF(maxNeg, 1); Serial.println();
  if (maxPos > 20 && maxNeg < -20) Serial.println(F("  Thay ca 2 chieu xoay. Xoay TRAI phai ra so DUONG (+); neu nguoc -> GYRO_SIGN = -1"));
  else Serial.println(F("  Chua thay xoay ro rang (ban co xoay xe khong?)"));
  setResult(4, okCount >= 9 && accOk ? PASS : FAIL);
}

void testOneVL53(int i) {
  const int k = 5 + i;
  Serial.print(F("\r\n[")); Serial.print(k); Serial.print(F("] TEST RIENG ")); Serial.print(VL_NAME[i]);
  Serial.println(F(": chi bat cam bien nay, 3 cam bien kia tat (XSHUT LOW)."));
  allXshutLow();
  delay(20);
  if (!bootOne(i)) { setResult(k, FAIL); return; }
  Serial.println(F("  Dua tay/hop truoc cam bien o 10..30 cm, roi rut ra. 10 lan doc, 1 s / lan."));
  int valid = 0, near = 0, tout = 0; uint16_t mn = 9999, mx = 0;
  for (int s = 1; s <= 10; ++s) {
    if (!waitMs(1000)) return;
    uint16_t mm; const int st = readRange(i, mm);
    if (st == 1) ++tout; else { ++valid; if (st == 0) { ++near; mn = min(mn, mm); mx = max(mx, mm); } }
    Serial.print(F("  ")); Serial.print(s); Serial.print(F("s: ")); printRange(st, mm); Serial.println();
  }
  Serial.print(F("  Hop le ")); Serial.print(valid); Serial.print(F("/10, timeout ")); Serial.print(tout);
  if (near) { Serial.print(F(", khoang ")); Serial.print(mn); Serial.print(F("..")); Serial.print(mx); Serial.print(F(" mm")); }
  Serial.println();
  if (!near) Serial.println(F("  Khong lan nao thay vat -> dua tay gan hon hoac kiem tra cam bien bi che/ban"));
  setResult(k, valid >= 9 && near >= 1 ? PASS : FAIL);
}

void testAllVL53() {
  Serial.println(F("\r\n[9] TEST 4 VL53 CUNG BAT: khoi tao tuan tu XSHUT, doi dia chi 0x30..0x33, doc ca 4 moi giay."));
  if (!bootAll()) { setResult(9, FAIL); return; }
  Serial.println(F("  Dua tay truoc TUNG cam bien de thay so do cua dung cam bien do thay doi (kiem tra cam nham vi tri)."));
  int valid[4] = {0, 0, 0, 0};
  for (int s = 1; s <= 10; ++s) {
    if (!waitMs(1000)) return;
    Serial.print(F("  ")); Serial.print(s); Serial.print(F("s:"));
    for (int i = 0; i < 4; ++i) {
      Serial.print(i == 0 ? F(" J8=") : i == 1 ? F("  J9=") : i == 2 ? F("  J10=") : F("  J11="));
      if (!tofReady[i]) { Serial.print(F("LOI")); continue; }
      uint16_t mm; const int st = readRange(i, mm);
      if (st != 1) ++valid[i];
      printRange(st, mm);
    }
    Serial.println();
  }
  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    Serial.print(F("  ")); Serial.print(VL_NAME[i]); Serial.print(F(": hop le ")); Serial.print(valid[i]); Serial.println(F("/10"));
    if (valid[i] < 9) ok = false;
  }
  setResult(9, ok ? PASS : FAIL);
}

// Chay motor trong ms, in xung/giay moi 1 s. Tra ve tong xung (da ap dau) cua 2 banh va gyro trung binh.
bool runMotors(int pwmL, int pwmR, uint32_t ms, int32_t out[2], float &gyroAvg) {
  readEnc();
  const int32_t s0[2] = {enc[0], enc[1]};
  int32_t last[2] = {enc[0], enc[1]};
  float gsum = 0; int gn = 0;
  motorSet(0, pwmL); motorSet(1, pwmR);
  const uint32_t t0 = millis();
  uint32_t nextPrint = 1000;
  while (millis() - t0 < ms) {
    float z;
    if (imuOk && millis() - t0 > 250 && readGyroZ(z)) { gsum += z; ++gn; }
    if (!waitMs(20)) return false;
    if (millis() - t0 >= nextPrint) {
      nextPrint += 1000;
      readEnc();
      Serial.print(F("     ")); Serial.print((millis() - t0) / 1000); Serial.print(F("s: TRAI ")); printSigned(enc[0] - last[0]);
      Serial.print(F(" xung/s   PHAI ")); printSigned(enc[1] - last[1]); Serial.print(F(" xung/s"));
      if (imuOk && gn) { Serial.print(F("   gyroZ ")); printSignedF(gsum / gn, 1); Serial.print(F(" do/s")); }
      Serial.println();
      last[0] = enc[0]; last[1] = enc[1];
    }
  }
  motorBrake();
  if (!waitMs(400)) return false;
  motorsOff();
  readEnc();
  out[0] = enc[0] - s0[0]; out[1] = enc[1] - s0[1];
  gyroAvg = gn ? gsum / gn : 0;
  return true;
}
// Kiem tra dau 1 banh: expect +1 tien, -1 lui, 0 dung
bool checkWheel(const __FlashStringHelper *name, int32_t d, int expect) {
  Serial.print(F("     ")); Serial.print(name); Serial.print(F(": ")); printSigned(d); Serial.print(F(" xung -> "));
  if (expect == 0) { const bool ok = abs(d) < 30; Serial.println(ok ? F("dung yen OK") : F("LOI: khong chay ma co xung")); return ok; }
  if (abs(d) < 60) { Serial.println(F("LOI: qua it xung (motor khong quay / encoder mat / PWM thap, thu '+')")); return false; }
  if ((d > 0 ? 1 : -1) != expect) {
    Serial.println(F("LOI SAI CHIEU: nhin banh - neu banh quay DUNG chieu thi encoder dao dau, neu banh quay NGUOC thi day motor dao"));
    return false;
  }
  Serial.println(F("OK dung chieu"));
  return true;
}
float diffPct(int32_t a, int32_t b) {
  const float s = fabsf(float(a)) + fabsf(float(b));
  return s > 0 ? 200.0f * fabsf(fabsf(float(a)) - fabsf(float(b))) / s : 0;
}

void testWheels() {
  Serial.print(F("\r\n[a] TEST MOTOR + ENCODER TUNG BANH, PWM ")); Serial.println(testPwm);
  Serial.println(F("  !!! NANG BANH XE KHOI MAT DAT. Bat dau sau 3 s (go x de huy)"));
  if (!waitMs(3000)) return;
  struct Step { const __FlashStringHelper *name; int l, r; };
  const Step steps[4] = {{F("TRAI TIEN"), 1, 0}, {F("TRAI LUI"), -1, 0}, {F("PHAI TIEN"), 0, 1}, {F("PHAI LUI"), 0, -1}};
  bool ok = true;
  int32_t fwd[2] = {0, 0};
  for (const Step &st : steps) {
    Serial.print(F("  >> ")); Serial.println(st.name);
    int32_t d[2]; float g;
    if (!runMotors(st.l * testPwm, st.r * testPwm, 2000, d, g)) return;
    ok &= checkWheel(F("Banh TRAI"), d[0], st.l);
    ok &= checkWheel(F("Banh PHAI"), d[1], st.r);
    if (st.l > 0) fwd[0] = d[0];
    if (st.r > 0) fwd[1] = d[1];
  }
  if (fwd[0] > 0 && fwd[1] > 0) {
    Serial.print(F("  Cung PWM, tien 2 s: TRAI ")); Serial.print(fwd[0]); Serial.print(F(" / PHAI ")); Serial.print(fwd[1]);
    Serial.print(F(" xung, chenh ")); Serial.print(diffPct(fwd[0], fwd[1]), 1); Serial.println(F(" %"));
  }
  setResult(10, ok ? PASS : FAIL);
}

// ---------------- [c] Encoder ca 2 banh cung chay ----------------
void testBothEncoders() {
  Serial.print(F("\r\n[c] TEST ENCODER CA 2 BANH CUNG CHAY, PWM ")); Serial.println(testPwm);
  Serial.println(F("  !!! NANG BANH XE. 2 banh cung TIEN 3 s, nghi, cung LUI 3 s. Bat dau sau 3 s (x de huy)"));
  if (!waitMs(3000)) return;
  bool ok = true;
  int32_t tien[2] = {0, 0}, lui[2] = {0, 0};
  for (int dir = 1; dir >= -1; dir -= 2) {
    Serial.println(dir > 0 ? F("  >> CA 2 BANH TIEN") : F("  >> CA 2 BANH LUI"));
    int32_t d[2]; float g;
    if (!runMotors(dir * testPwm, dir * testPwm, 3000, d, g)) return;
    ok &= checkWheel(F("Banh TRAI"), d[0], dir);
    ok &= checkWheel(F("Banh PHAI"), d[1], dir);
    const float p = diffPct(d[0], d[1]);
    Serial.print(F("     Chenh lech 2 banh: ")); Serial.print(p, 1); Serial.print(F(" %  "));
    Serial.println(p < 10 ? F("(binh thuong, PID se bu)") : p < 25 ? F("(LON - kiem tra banh/ma sat/encoder)") : F("(QUA LON - 1 encoder dem sai hoac 1 motor yeu)"));
    if (p >= 25) ok = false;
    if (dir > 0) { tien[0] = d[0]; tien[1] = d[1]; } else { lui[0] = d[0]; lui[1] = d[1]; }
    if (!waitMs(500)) return;
  }
  // Moi banh: so xung tien va lui phai gan bang nhau (encoder khong mat xung theo 1 chieu)
  for (int i = 0; i < 2; ++i) {
    const float p = diffPct(tien[i], lui[i]);
    Serial.print(i ? F("  Banh PHAI tien/lui: ") : F("  Banh TRAI tien/lui: "));
    Serial.print(tien[i]); Serial.print('/'); Serial.print(lui[i]); Serial.print(F("  lech ")); Serial.print(p, 1);
    Serial.println(p < 15 ? F(" % OK") : F(" % LECH NHIEU (mat xung 1 chieu? day encoder long?)"));
  }
  setResult(12, ok ? PASS : FAIL);
}

// ---------------- [d] Tat ca linh kien hoat dong cung luc ----------------
void testEverything() {
  Serial.print(F("\r\n[d] TEST TAT CA LINH KIEN CUNG LUC 15 s, PWM ")); Serial.println(testPwm);
  Serial.println(F("  2 motor chay tien + 4 VL53 do lien tuc + gyro + encoder + SW1 + LED."));
  Serial.println(F("  !!! NANG BANH XE. Trong luc chay: DUA TAY truoc cac VL53, NHAN SW1 vai lan."));
  Serial.println(F("  Bai nay bat loi nhieu motor lam treo I2C / mat cam bien / encoder dem sai."));
  uint8_t who;
  imuOk = initImu(who);
  Serial.println(imuOk ? F("  MPU6050 OK") : F("  MPU6050 LOI"));
  const int booted = bootAll();
  if (!waitMs(2000)) return;
  // thong ke
  uint32_t frames[4] = {0, 0, 0, 0}, i2cErr[4] = {0, 0, 0, 0}, frameSec[4] = {0, 0, 0, 0};
  uint32_t lastFrame[4], maxGap[4] = {0, 0, 0, 0};
  uint16_t lastMm[4] = {0, 0, 0, 0};
  uint32_t gyroOk = 0, gyroErr = 0, maxLoopUs = 0, loops = 0;
  int32_t cpsMin[2] = {INT32_MAX, INT32_MAX};
  float gyroSum = 0; int gyroN = 0;
  swPress = swRelease = 0;
  readEnc();
  int32_t lastEnc[2] = {enc[0], enc[1]};
  const uint32_t t0 = millis();
  for (int i = 0; i < 4; ++i) lastFrame[i] = t0;
  uint32_t nextPrint = t0 + 1000, nextGyro = t0, nextPoll = t0;
  int pollIdx = 0;
  motorSet(0, testPwm); motorSet(1, testPwm);
  Serial.println(F("  giay | ENC TRAI  PHAI (xung/s) | GYRO dps | J8   J9   J10  J11 (mm, mau/s) | SW1 | loop max"));
  while (millis() - t0 < 15000) {
    const uint32_t us0 = micros();
    const uint32_t now = millis();
    serviceButton();
    digitalWrite(LED_PIN, (now % 200) < 100 ? LOW : HIGH);
    while (Serial.available()) {
      const char c = char(Serial.read());
      if (c == 'x' || c == 'X') { motorsOff(); aborted = true; Serial.println(F("\r\n!!! DA HUY BAI TEST")); return; }
    }
    if (imuOk && now >= nextGyro) {
      nextGyro = now + 10;
      float z;
      if (readGyroZ(z)) { ++gyroOk; gyroSum += z; ++gyroN; } else ++gyroErr;
    }
    if (now >= nextPoll) {                 // moi 2 ms doc 1 cam bien (khong chan)
      nextPoll = now + 2;
      const int i = pollIdx; pollIdx = (pollIdx + 1) % 4;
      if (tofReady[i]) {
        uint16_t mm;
        const int r = pollRange(i, mm);
        if (r < 0) ++i2cErr[i];
        else if (r > 0) {
          ++frames[i]; ++frameSec[i]; lastMm[i] = mm;
          if (now - lastFrame[i] > maxGap[i]) maxGap[i] = now - lastFrame[i];
          lastFrame[i] = now;
        }
      }
    }
    if (now >= nextPrint) {
      nextPrint += 1000;
      readEnc();
      const int32_t d0 = enc[0] - lastEnc[0], d1 = enc[1] - lastEnc[1];
      lastEnc[0] = enc[0]; lastEnc[1] = enc[1];
      if (now - t0 > 1500) { cpsMin[0] = min(cpsMin[0], d0); cpsMin[1] = min(cpsMin[1], d1); }
      Serial.print(F("  ")); if ((now - t0) / 1000 < 10) Serial.print(' '); Serial.print((now - t0) / 1000);
      Serial.print(F("s  | ")); printSigned(d0); Serial.print(F("  ")); printSigned(d1);
      Serial.print(F(" | ")); if (gyroN) printSignedF(gyroSum / gyroN, 1); else Serial.print(F("LOI"));
      Serial.print(F(" |"));
      for (int i = 0; i < 4; ++i) {
        Serial.print(' ');
        if (!tofReady[i]) { Serial.print(F("LOI")); continue; }
        Serial.print(lastMm[i] >= 8190 ? 0 : lastMm[i]); Serial.print('('); Serial.print(frameSec[i]); Serial.print(')');
        frameSec[i] = 0;
      }
      Serial.print(F(" | ")); Serial.print(swPress);
      Serial.print(F(" | ")); Serial.print(maxLoopUs); Serial.println(F("us"));
      gyroSum = 0; gyroN = 0;
    }
    ++loops;
    const uint32_t dt = micros() - us0;
    if (dt > maxLoopUs) maxLoopUs = dt;
  }
  motorBrake();
  waitMs(400);
  motorsOff();
  digitalWrite(LED_PIN, HIGH);

  Serial.println(F("  ---- TONG KET ----"));
  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    Serial.print(F("  ")); Serial.print(VL_NAME[i]); Serial.print(F(": "));
    if (!tofReady[i]) { Serial.println(F("KHONG khoi tao duoc -> FAIL")); ok = false; continue; }
    const float rate = frames[i] / 15.0f;
    Serial.print(rate, 1); Serial.print(F(" mau/s, khoang trong lon nhat ")); Serial.print(maxGap[i]);
    Serial.print(F(" ms, loi I2C ")); Serial.print(i2cErr[i]);
    const bool good = rate >= 20 && maxGap[i] < 300 && i2cErr[i] == 0;
    Serial.println(good ? F(" -> OK") : F(" -> LOI (nhieu motor / day long / nguon yeu)"));
    ok &= good;
  }
  Serial.print(F("  Gyro: doc OK ")); Serial.print(gyroOk); Serial.print(F(", loi ")); Serial.print(gyroErr);
  Serial.println(gyroErr == 0 && gyroOk > 1000 ? F(" -> OK") : F(" -> LOI"));
  ok &= gyroErr == 0 && gyroOk > 1000;
  for (int i = 0; i < 2; ++i) {
    Serial.print(i ? F("  Encoder PHAI: thap nhat ") : F("  Encoder TRAI: thap nhat "));
    Serial.print(cpsMin[i]); Serial.print(F(" xung/s"));
    const bool good = cpsMin[i] != INT32_MAX && cpsMin[i] > 50;
    Serial.println(good ? F(" -> OK (khong mat xung)") : F(" -> LOI: co giay bi mat xung / dung"));
    ok &= good;
  }
  Serial.print(F("  SW1 nhan ")); Serial.print(swPress); Serial.println(swPress ? F(" lan -> OK") : F(" lan (ban chua nhan - khong tinh loi)"));
  Serial.print(F("  Vong lap: ")); Serial.print(loops); Serial.print(F(" lan, lau nhat ")); Serial.print(maxLoopUs);
  Serial.println(maxLoopUs < 20000 ? F(" us -> OK") : F(" us -> CHAM (I2C bi cho/treo)"));
  ok &= maxLoopUs < 20000;
  if (booted < 4) Serial.println(F("  (co VL53 khong khoi tao duoc)"));
  setResult(13, ok ? PASS : FAIL);
}

void testDrive() {
  Serial.print(F("\r\n[b] TEST XE CHAY THAT (encoder + gyro), PWM ")); Serial.println(testPwm);
  Serial.println(F("  !!! DAT XE XUONG SAN, khoang trong 50 cm. Thu tu: TIEN, LUI, QUAY TRAI, QUAY PHAI (moi buoc 2 s)."));
  uint8_t who;
  if (!imuOk) imuOk = initImu(who);
  if (!imuOk) Serial.println(F("  (MPU6050 loi: chi kiem tra encoder, khong kiem tra chieu quay bang gyro)"));
  float bias = 0;
  if (imuOk) {
    Serial.println(F("  Do bias gyro 2 s, de xe dung yen..."));
    float s = 0; int n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 2000) { float z; if (readGyroZ(z)) { s += z; ++n; } if (!waitMs(10)) return; }
    bias = n ? s / n : 0;
    Serial.print(F("  bias = ")); printSignedF(bias, 2); Serial.println(F(" do/s"));
  }
  Serial.println(F("  Bat dau sau 3 s (go x de huy)"));
  if (!waitMs(3000)) return;
  // l, r: dau mong doi cua encoder; gz: dau mong doi cua gyro (+ = quay trai, nguoc kim dong ho)
  struct Step { const __FlashStringHelper *name; int l, r, gz; };
  const Step steps[4] = {{F("TIEN"), 1, 1, 0}, {F("LUI"), -1, -1, 0}, {F("QUAY TRAI tai cho"), -1, 1, 1}, {F("QUAY PHAI tai cho"), 1, -1, -1}};
  bool ok = true, gyroSignWrong = false;
  for (const Step &st : steps) {
    Serial.print(F("  >> ")); Serial.println(st.name);
    int32_t d[2]; float g;
    if (!runMotors(st.l * testPwm, st.r * testPwm, 2000, d, g)) return;
    g -= bias;
    ok &= checkWheel(F("Banh TRAI"), d[0], st.l);
    ok &= checkWheel(F("Banh PHAI"), d[1], st.r);
    if (imuOk) {
      Serial.print(F("     Gyro Z trung binh: ")); printSignedF(g, 1); Serial.print(F(" do/s -> "));
      if (st.gz == 0) {
        if (fabsf(g) < 15) Serial.println(F("chay gan thang OK"));
        else Serial.println(g > 0 ? F("xe lech sang TRAI (banh phai nhanh hon) - binh thuong khi chua co PID") : F("xe lech sang PHAI (banh trai nhanh hon) - binh thuong khi chua co PID"));
      } else if (fabsf(g) < 15) { Serial.println(F("LOI: gyro khong thay xe quay (xe bi ket? MPU loi?)")); ok = false; }
      else if ((g > 0 ? 1 : -1) != st.gz) { Serial.println(F("SAI DAU: gyro nguoc chieu -> dat GYRO_SIGN = -1 trong firmware")); gyroSignWrong = true; }
      else Serial.println(F("OK dung chieu"));
    }
    if (st.gz == 0 && d[0] && d[1]) { Serial.print(F("     Chenh lech 2 banh: ")); Serial.print(diffPct(d[0], d[1]), 1); Serial.println(F(" %")); }
  }
  if (gyroSignWrong) Serial.println(F("  => Encoder dung nhung dau gyro nguoc: firmware tu hanh can GYRO_SIGN = -1"));
  setResult(11, ok && !gyroSignWrong ? PASS : FAIL);
}

void printMenu() {
  Serial.println(F("\r\n================= MENU TEST LINH KIEN ================="));
  Serial.println(F(" 1 SW1              2 LED              3 Encoder quay tay   4 MPU6050"));
  Serial.println(F(" 5 VL53 J8 rieng    6 VL53 J9 rieng    7 VL53 J10 rieng     8 VL53 J11 rieng"));
  Serial.println(F(" 9 4 VL53 cung bat 1 luc"));
  Serial.println(F(" a Motor+encoder TUNG banh      c Encoder CA 2 BANH cung chay     (NANG BANH)"));
  Serial.println(F(" d TAT CA linh kien cung luc 15 s (NANG BANH)"));
  Serial.println(F(" b Xe chay tien/lui/trai/phai (DAT XUONG SAN)"));
  Serial.println(F(" 0 Chay tat ca lan luot   r Bang ket qua   + / - PWM   x Huy   h Menu"));
  Serial.print(F(" PWM thu hien tai = ")); Serial.println(testPwm);
}

char keyOf(int k) { return k <= 9 ? char('0' + k) : "abcd"[k - 10]; }
void printSummary() {
  Serial.println(F("\r\n================= BANG KET QUA ================="));
  for (int k = 1; k < N_TEST; ++k) {
    Serial.print(F(" ")); Serial.print(keyOf(k)); Serial.print(F("  "));
    Serial.print(TEST_NAME[k]); Serial.print(F(": "));
    Serial.println(result[k] == PASS ? F("PASS") : result[k] == FAIL ? F("FAIL") : result[k] == CHECK ? F("xac nhan bang mat") : F("chua chay"));
  }
}

void runTest(char c) {
  aborted = false;
  motorsOff();
  switch (c) {
    case '1': testSW1(); break;
    case '2': testLED(); break;
    case '3': testEncoderHand(); break;
    case '4': testMPU(); break;
    case '5': case '6': case '7': case '8': testOneVL53(c - '5'); break;
    case '9': testAllVL53(); break;
    case 'a': case 'A': testWheels(); break;
    case 'c': case 'C': testBothEncoders(); break;
    case 'd': case 'D': testEverything(); break;
    case 'b': case 'B': testDrive(); break;
    case '0': {
      const char order[] = "123456789acdb";
      for (const char *p = order; *p && !aborted; ++p) {
        if (*p == 'a') Serial.println(F("\r\n*** Sap test motor: NANG BANH XE ***"));
        if (*p == 'b') { Serial.println(F("\r\n*** Tiep theo: DAT XE XUONG SAN, co 10 s chuan bi (x de dung) ***")); if (!waitMs(10000)) break; }
        runTest(*p);
      }
      printSummary();
      break;
    }
    default: return;
  }
  motorsOff();
  if (c != '0') Serial.println(F("\r\n(go h de xem menu, r de xem bang ket qua)"));
}

}  // namespace

void setup() {
  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, LOW);
  for (int i = 0; i < 2; ++i) { pinMode(IN1[i], OUTPUT); pinMode(IN2[i], OUTPUT); pinMode(PWM_PIN[i], OUTPUT); }
  analogWriteResolution(8);
  motorsOff();
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(SW1_PIN, INPUT_PULLUP);
  allXshutLow();
  Serial.setRx(PA10); Serial.setTx(PA9);
  Serial.begin(TEST_BAUD);
  Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(100000);
  bus2.begin(); bus2.setClock(100000);
  delay(300);
  Serial.println(F("\r\n\r\n### TEST LINH KIEN - STM32F103C8T6 ###"));
  Serial.print(F("Encoder TIM3/TIM2: ")); Serial.println(initEncoders() ? F("cau hinh OK") : F("LOI cau hinh timer"));
  uint8_t who;
  imuOk = initImu(who);
  printMenu();
}

void loop() {
  serviceButton();
  digitalWrite(LED_PIN, (millis() % 1000) < 50 ? LOW : HIGH);   // nhip tim
  if (Serial.available()) {
    const char c = char(Serial.read());
    if (c == 'h' || c == 'H' || c == '?') printMenu();
    else if (c == 'r' || c == 'R') printSummary();
    else if (c == '+') { testPwm = min(testPwm + 10, 160); Serial.print(F("PWM thu = ")); Serial.println(testPwm); }
    else if (c == '-') { testPwm = max(testPwm - 10, 40); Serial.print(F("PWM thu = ")); Serial.println(testPwm); }
    else if (c == 'x' || c == 'X') { motorsOff(); Serial.println(F("Motor TAT")); }
    else runTest(c);
  }
}