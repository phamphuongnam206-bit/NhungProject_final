#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

// STM32F103C8T6 / TB6612FNG: same pins as testEncoder.cpp.
// Bench test only. No map, IMU, or obstacle avoidance.
// Default: both wheels forward, 200 counts/s. Lift wheels before testing.
// Optional fixed pivot modes: only for bench testing signed wheel control.
namespace {
constexpr uint32_t STBY = PB0, SW1 = PB1, LED = PC13;
constexpr uint32_t L_IN1 = PB14, L_IN2 = PB15, L_PWM = PB9;
constexpr uint32_t R_IN1 = PB12, R_IN2 = PB13, R_PWM = PB8;
constexpr uint32_t BAUD = 115200;
constexpr uint32_t CONTROL_MS = 10, MAX_GAP_MS = 50;
constexpr uint32_t RUN_MS = 10000, LOG_MS = 200, DEBOUNCE_MS = 30;
// Change these two settings only between tests (recompile).
enum class TestMode : uint8_t { FORWARD, PIVOT_LEFT, PIVOT_RIGHT };
constexpr TestMode TEST_MODE = TestMode::FORWARD;
constexpr float TARGET_CPS = 200.0f; // Then test 400, 200, 2000 separately.
static_assert(TARGET_CPS >= 100.0f && TARGET_CPS <= 2000.0f, "Test target out of range");
constexpr int8_t LEFT_DIR = TEST_MODE == TestMode::PIVOT_LEFT ? -1 : 1;
constexpr int8_t RIGHT_DIR = TEST_MODE == TestMode::PIVOT_RIGHT ? -1 : 1;
constexpr int8_t LEFT_ENCODER_SIGN = -1, RIGHT_ENCODER_SIGN = 1;
constexpr float ACCEL_CPS_S = TARGET_CPS <= 1000 ? 700.0f : 2000.0f;
constexpr float DECEL_CPS_S = TARGET_CPS <= 1000 ? 900.0f : 2500.0f;
constexpr int PWM_MAX = 160;
constexpr float PWM_REF = 160.0f, LEFT_REF = 5503.96f, RIGHT_REF = 5298.09f;
// Provisional tuning, not calibrated hardware values.
constexpr float FF_OFFSET = 42.0f, FF_BLEND_CPS = 1000.0f;
constexpr float KP_L = 0.018f, KI_L = 0.08f;
constexpr float KP_R = 0.018f, KI_R = 0.08f;
constexpr int ACTIVE_PWM_MAX = TARGET_CPS <= 1000 ? 85 : PWM_MAX;
constexpr float FILTER_TAU = 0.040f, I_MAX = 65.0f;
// Provisional low-speed protection; applies even below 500 CPS.
constexpr float STALL_MIN_TARGET = 100.0f, STALL_MIN_CPS = 20.0f;
constexpr uint32_t STALL_MS = 1500, REVERSE_MS = 100;
constexpr float REVERSE_CPS = 60.0f;
constexpr uint32_t QUIET_MS = 300, COAST_LIMIT_MS = 2500;
constexpr uint32_t ENC_FILTER = 8;

struct Wheel {
    uint16_t prev = 0;
    int32_t total = 0;
    int8_t sign = 0; // Fixed raw encoder polarity, set at startRun().
    float speed = 0, integral = 0;
    int pwm = 0;
    uint32_t stall = 0, reverse = 0;
};
Wheel left, right;
enum class State : uint8_t { WAIT, RUN, RAMP_DOWN, COAST, FINISH, FAULT };
State state = State::WAIT;
uint32_t lastTick = 0, runStart = 0, lastLog = 0, stateSince = 0;
uint32_t lastMotion = 0, buttonChanged = 0;
bool rawButton = false, stableButton = false, armed = false, completed = false;
float target = 0;
const char *fault = "NONE";
char logBuf[320];
size_t logLen = 0, logPos = 0;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
void led(bool on) { digitalWrite(LED, on ? LOW : HIGH); }
void outLow(uint32_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
void disableDrive() {
    digitalWrite(STBY, LOW); // Coast/high impedance, not active brake.
    analogWrite(L_PWM, 0); analogWrite(R_PWM, 0);
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
    left.pwm = right.pwm = 0;
    left.integral = right.integral = 0;
}
void enableDrive() {
    // Never change direction under power. Fixed for the entire run.
    digitalWrite(STBY, LOW);
    digitalWrite(L_IN1, LEFT_DIR > 0 ? LOW : HIGH);
    digitalWrite(L_IN2, LEFT_DIR > 0 ? HIGH : LOW);
    digitalWrite(R_IN1, RIGHT_DIR > 0 ? HIGH : LOW);
    digitalWrite(R_IN2, RIGHT_DIR > 0 ? LOW : HIGH);
    analogWrite(L_PWM, 0); analogWrite(R_PWM, 0);
    digitalWrite(STBY, HIGH);
}
void configureTimer(TIM_TypeDef *t) {
    t->CR1 = 0; t->CR2 = 0; t->SMCR = 0; t->DIER = 0; t->CCER = 0;
    t->PSC = 0; t->ARR = 65535;
    t->CCMR1 = 1U | (ENC_FILTER << 4) | (1U << 8) | (ENC_FILTER << 12);
    t->CCMR2 = 0; t->SMCR = 3U;
    t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    t->EGR = TIM_EGR_UG; t->CNT = 0; t->SR = 0; t->CR1 = TIM_CR1_CEN;
}
bool timerOK(TIM_TypeDef *t) {
    return (t->CR1 & TIM_CR1_CEN) && (t->SMCR & 7U) == 3U &&
        t->PSC == 0 && t->ARR == 65535 && t->DIER == 0 &&
        (t->CCMR1 & 0xFFFFU) == (1U | (ENC_FILTER << 4) | (1U << 8) | (ENC_FILTER << 12)) &&
        (t->CCER & 0xFFFFU) == (TIM_CCER_CC1E | TIM_CCER_CC2E);
}
bool initEncoders() {
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
    __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn); HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
    AFIO->MAPR = mapr | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    pinMode(PA6, INPUT_PULLUP); pinMode(PA7, INPUT_PULLUP);
    pinMode(PA0, INPUT_PULLUP); pinMode(PA1, INPUT_PULLUP);
    configureTimer(TIM2); configureTimer(TIM3);
    return !(AFIO->MAPR & (AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP)) && timerOK(TIM2) && timerOK(TIM3);
}
int32_t delta16(uint16_t now, uint16_t prev) {
    uint16_t d = static_cast<uint16_t>(now - prev);
    return d < 32768U ? static_cast<int32_t>(d) : static_cast<int32_t>(d) - 65536L;
}
int32_t sample(Wheel &w, uint16_t count, uint32_t ms, bool driving, int8_t dir) {
    const int32_t d = delta16(count, w.prev);
    w.prev = count; w.total += d;
    const float cps = w.sign ? (1000.0f * d * w.sign / ms) : 0.0f;
    const float dt = ms * 0.001f;
    w.speed += dt / (FILTER_TAU + dt) * (cps - w.speed);
    if (driving && w.speed * dir < -REVERSE_CPS) w.reverse += ms;
    else w.reverse = 0;
    return d;
}
int pi(Wheel &w, float sp, float ref, float kp, float ki, float dt, int8_t dir) {
    if (sp <= 0) { w.integral = 0; return 0; }
    // Fade offset to zero at low speed; preserve old FF above 1000 CPS.
    const float blend = clampf(sp / FF_BLEND_CPS, 0.0f, 1.0f);
    const float ff = FF_OFFSET * blend + (PWM_REF - FF_OFFSET) * sp / ref;
    const float error = sp - w.speed * dir;
    const float nextI = clampf(w.integral + ki * error * dt, -I_MAX, I_MAX);
    const float proposed = ff + kp * error + nextI;
    // Conditional integration: prevent windup into either output limit.
    if ((proposed >= 0 && proposed <= ACTIVE_PWM_MAX) || (proposed > ACTIVE_PWM_MAX && error < 0) || (proposed < 0 && error > 0)) w.integral = nextI;
    return static_cast<int>(clampf(ff + kp * error + w.integral, 0, ACTIVE_PWM_MAX) + 0.5f);
}
void queueText(const char *s) {
    // Never wait for UART. An event replaces an unsent/partial sample.
    int n = snprintf(logBuf, sizeof(logBuf), "\n%s\n", s);
    logPos = 0; logLen = n > 0 ? static_cast<size_t>(n) : 0;
    if (logLen >= sizeof(logBuf)) logLen = sizeof(logBuf) - 1;
}
void flushLog() {
    if (logPos >= logLen) return;
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    size_t n = logLen - logPos;
    if (n > static_cast<size_t>(room)) n = static_cast<size_t>(room);
    if (n > 32) n = 32;
    logPos += Serial.write(reinterpret_cast<const uint8_t *>(logBuf + logPos), n);
}
const char *stateName() {
    switch (state) {
        case State::WAIT: return "WAIT"; case State::RUN: return "RUN";
        case State::RAMP_DOWN: return "RAMP_DOWN"; case State::COAST: return "COAST";
        case State::FINISH: return "FINISH"; default: return "FAULT";
    }
}
void telemetry(uint32_t now) {
    if (now - lastLog < LOG_MS || logPos < logLen) return;
    lastLog = now;
    const float a = fabsf(left.speed), b = fabsf(right.speed);
    const long diff100 = a+b >= 200.0f ? lroundf(20000.0f*fabsf(a-b)/(a+b)) : -1;
    int n = snprintf(logBuf, sizeof(logBuf), "%lu,%s,%ld,%ld,%ld,%d,%d,%ld,%ld,%d,%d,%ld,%s,%ld,%ld\n",
        (unsigned long)(now-runStart), stateName(), lroundf(target), lroundf(left.speed), lroundf(right.speed),
        left.pwm, right.pwm, (long)left.total, (long)right.total, (int)left.sign, (int)right.sign, diff100, fault, lroundf(target * LEFT_DIR), lroundf(target * RIGHT_DIR));
    logPos = 0; logLen = n > 0 ? static_cast<size_t>(n) : 0;
    if (logLen >= sizeof(logBuf)) logLen = sizeof(logBuf)-1;
}
void fail(const char *why, uint32_t now) {
    disableDrive(); target = 0; fault = why; state = State::FAULT; stateSince = now;
    queueText(why); // Latched fault; RESET required after inspection.
}
void waitState(uint32_t now) {
    disableDrive(); target = 0; state = State::WAIT; stateSince = now; armed = false;
    queueText("WAIT_SW1");
}
void startRun(uint32_t now) {
    if (!timerOK(TIM2) || !timerOK(TIM3)) { fail("ENCODER_CONFIG_ERROR", now); return; }
    left = Wheel{}; right = Wheel{};
    left.sign = LEFT_ENCODER_SIGN; right.sign = RIGHT_ENCODER_SIGN;
    left.prev = static_cast<uint16_t>(TIM3->CNT); right.prev = static_cast<uint16_t>(TIM2->CNT);
    runStart = lastTick = lastLog = stateSince = now;
    target = 0; completed = false; fault = "NONE";
    enableDrive(); state = State::RUN; led(true);
    queueText("START\nTIME_MS,STATE,TARGET_CPS,ACTUAL_CPS_L,ACTUAL_CPS_R,PWM_L,PWM_R,RAW_TOTAL_L,RAW_TOTAL_R,RAW_SIGN_L,RAW_SIGN_R,DIFF_PERCENT_X100,FAULT,TARGET_CPS_L,TARGET_CPS_R");
}
void beginCoast(uint32_t now, bool success) {
    disableDrive(); target = 0; completed = success;
    state = State::COAST; stateSince = lastMotion = now;
    queueText(success ? "RAMP_COMPLETE_COAST" : "MANUAL_STOP_COAST");
}
bool pressed(uint32_t now) {
    bool v = digitalRead(SW1) == LOW;
    if (v != rawButton) { rawButton = v; buttonChanged = now; }
    if (now - buttonChanged >= DEBOUNCE_MS) {
        if (!rawButton && !stableButton) armed = true;
        if (stableButton != rawButton) {
            stableButton = rawButton;
            if (!stableButton) armed = true;
            else if (armed) { armed = false; return true; }
        }
    }
    return false;
}
void control(uint32_t now) {
    const uint32_t ms = now - lastTick;
    if (ms < CONTROL_MS) return;
    if (ms > MAX_GAP_MS) { fail("CONTROL_TIMEOUT", now); return; }
    lastTick = now;
    bool driving = state == State::RUN || state == State::RAMP_DOWN;
    int32_t dl = sample(left, static_cast<uint16_t>(TIM3->CNT), ms, driving, LEFT_DIR);
    int32_t dr = sample(right, static_cast<uint16_t>(TIM2->CNT), ms, driving, RIGHT_DIR);
    // A very large count jump at this test speed indicates unreliable data.
    if (driving && (dl > 30000 || dl < -30000 || dr > 30000 || dr < -30000 ||
        fabsf(1000.0f*dl/ms) > 20000 || fabsf(1000.0f*dr/ms) > 20000)) {
        fail("ENCODER_RATE_INVALID", now); return;
    }
    if (state == State::COAST) {
        if (dl || dr) lastMotion = now;
        if (now - lastMotion >= QUIET_MS) {
            if (completed) { state = State::FINISH; stateSince = now; queueText("TEST_COMPLETE"); }
            else waitState(now);
        } else if (now - stateSince >= COAST_LIMIT_MS) fail("COAST_TIMEOUT", now);
        return;
    }
    if (!driving) return;
    if (state == State::RUN && now - runStart >= RUN_MS) state = State::RAMP_DOWN;
    const float dt = ms * 0.001f;
    if (state == State::RUN) target = clampf(target + ACCEL_CPS_S * dt, 0, TARGET_CPS);
    else target = clampf(target - DECEL_CPS_S * dt, 0, TARGET_CPS);
    if (target <= 0) { beginCoast(now, true); return; }
    if (left.reverse >= REVERSE_MS || right.reverse >= REVERSE_MS) { fail("ENCODER_REVERSE", now); return; }
    // Detect lack of progress at low or high speed, independently of PWM.
    // Do not trip on the expected final ramp-down; no re-enable on a fault.
    auto stalled = [ms](Wheel &w, int8_t dir) {
        const float minSpeed = fmaxf(STALL_MIN_CPS, target * 0.15f);
        if (state == State::RUN && target >= STALL_MIN_TARGET &&
            w.speed * dir < minSpeed) w.stall += ms;
        else w.stall = 0;
        return w.stall >= STALL_MS;
    };
    if (stalled(left, LEFT_DIR)) { fail("STALL_OR_ENCODER_LEFT", now); return; }
    if (stalled(right, RIGHT_DIR)) { fail("STALL_OR_ENCODER_RIGHT", now); return; }
    left.pwm = pi(left, target, LEFT_REF, KP_L, KI_L, dt, LEFT_DIR);
    right.pwm = pi(right, target, RIGHT_REF, KP_R, KI_R, dt, RIGHT_DIR);
    analogWrite(L_PWM, left.pwm); analogWrite(R_PWM, right.pwm);
}
} // namespace

void setup() {
    outLow(STBY); outLow(L_IN1); outLow(L_IN2); outLow(R_IN1); outLow(R_IN2); outLow(L_PWM); outLow(R_PWM);
    digitalWrite(LED, HIGH); pinMode(LED, OUTPUT); pinMode(SW1, INPUT_PULLUP);
    analogWriteResolution(8); disableDrive();
    Serial.setRx(PA10); Serial.setTx(PA9); Serial.begin(BAUD, SERIAL_8N1);
    uint32_t now = millis();
    rawButton = stableButton = digitalRead(SW1) == LOW; buttonChanged = now;
    if (!initEncoders()) { fail("ENCODER_CONFIG_ERROR", now); return; }
    waitState(now);
}
void loop() {
    uint32_t now = millis();
    bool press = pressed(now);
    switch (state) {
        case State::WAIT:
            led(((now-stateSince)/250U)%2U == 0);
            if (press) startRun(now);
            break;
        case State::RUN: case State::RAMP_DOWN:
            led(true);
            if (press) beginCoast(now, false);
            control(now); telemetry(now);
            break;
        case State::COAST:
            led(true); control(now); telemetry(now); break;
        case State::FINISH: {
            const uint32_t elapsed = now - stateSince;
            // Initial OFF 200 ms makes four flashes distinct from RUN.
            if (elapsed >= 1800U) waitState(now);
            else led(elapsed >= 200U && ((elapsed-200U)/200U)%2U == 0);
            break;
        }
        case State::FAULT: led(((now-stateSince)/100U)%2U == 0); telemetry(now); break;
    }
    flushLog();
}
