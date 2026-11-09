/*
  V5.3 DIAGNOSTIC ONLY: alternate realRPM and Pxxx duty; not competition display.
  Corrected RevL PCB: Nano mounts on the COMPONENT side, USB points LEFT.
  Old RevJ has different physical socket mapping; follow its verified adapter/orientation.
  SG90 POSITION servos; THESE units rated 3..7.2V per user: retain board 6V rail.
  Receiver CH1 -> J5/D2 for steering. CH2 throttle is not used.
  PowerON + stable steering500ms -> automatic soft start (~2s reference ramp). PowerOFF stops power.
  A runtime fault or lost steering latches STOP until an actual power cycle.
  Display CLK -> A4, DIO -> A5, VCC -> 5V, GND -> GND.
  Encoder J9: 1=GND, 2=5V, 3=A/D7, 4=B/D12 (B not counted).
  Confirm encoder wiring and 5V compatibility before connecting it.
  User calibration: 1019 A rising edges over 10 output-shaft turns (2026-10-19).
  Assumes the calibrated output shaft and propeller shaft rotate 1:1.
  TEST_MODE=0: automatic fixed-speed run. TEST_MODE=1: servos ONLY, motor OFF.
  Disconnect propeller and mechanical linkages for initial tests.
*/
#include <Arduino.h>
#include <Servo.h>
#include <util/atomic.h>
#include <avr/interrupt.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "Select classic Nano ATmega328P, 16MHz (new or old bootloader)."
#endif

// ---------------- User settings: everything is in THIS file ----------------
#ifndef TEST_MODE
#define TEST_MODE 0  // 0 = POWER-ON AUTO RUN; 1 = servo-only offline test
#endif
#ifndef ENABLE_BATTERY_CUTOFF
#define ENABLE_BATTERY_CUTOFF 0 // Enable only AFTER checking/calibrating A0 voltage.
#endif
// Count A-channel RISING edges over a known number of PROPELLER revolutions.
// If N rising edges were measured over K propeller revolutions, use NUM=N,
// DEN=K. The provided "22 CPR" sheet does not define its counting convention.
// A vendor's motor-shaft PPR must also include the actual gearbox ratio.
#ifndef ENCODER_PULSES_NUM
#define ENCODER_PULSES_NUM 1019UL // User-reported count increase over 10 turns.
#endif
#ifndef ENCODER_TURNS_DEN
#define ENCODER_TURNS_DEN 10UL
#endif
#ifndef ENCODER_CALIBRATION_MODE
#define ENCODER_CALIBRATION_MODE 0 // 1 = motor/servo OFF; display raw rising-edge CNT.
#endif
const uint32_t ENCODER_MIN_EDGE_US = 80;
const uint32_t ENCODER_STOP_TIMEOUT_US = 1500000UL;
const uint16_t TARGET_RPM = 600;
const uint16_t RPM_BAND_LOW = 570, RPM_BAND_HIGH = 630; // User:600 +/-5%.
const uint16_t OVERSPEED_CUT_RPM = 610; // Early cut; actual inertia can exceed this.
const uint16_t SPEED_ACCEL_RPM_PER_SEC = 300; // 0->600 reference takes about2s.
const uint16_t SPEED_BAND_HOLD_MS = 1000, SPEED_BAND_FAIL_MS = 3000;
const uint16_t ENCODER_LOSS_MS = 180, ENCODER_VERIFY_MS = 900;
const uint16_t STARTUP_HARD_MS = 3000; // Absolute no-feedback drive-time bound.
const uint8_t STARTUP_VERIFY_PWM = 100; // Start short verification at this duty.
const uint8_t STARTUP_PWM_MAX = 140;
// Q8 PI: same feedforward and Kp; Ki=6/256 per20ms corrects load faster.
// Positive integration remains blocked during ramp and output saturation.
const int16_t SPEED_FF_Q8 = 82, SPEED_KP_Q8 = 51, SPEED_KI_STEP_Q8 = 6;
const uint8_t DISPLAY_BRIGHTNESS = 2; // 0..7; moderate brightness for first test.
const bool MOTOR_ENABLED = true;
const bool MOTOR_REVERSED = false;
const bool STEERING_REVERSED = false;
const bool LEFT_REVERSED = false;
const bool RIGHT_REVERSED = false;

const uint16_t STEER_MIN = 1000, STEER_CENTER = 1500, STEER_MAX = 2000;
const uint16_t STEER_DEAD = 25;
// Large travel trial: check each unloaded servo before connecting linkages.
// Pulse width is NOT a guaranteed physical angle. Reduce endpoints if binding.
const uint16_t LEFT_CENTER = 1500, LEFT_MIN = 600, LEFT_MAX = 2400;
const uint16_t RIGHT_CENTER = 1500, RIGHT_MIN = 600, RIGHT_MAX = 2400;
// Quick follow: move 3/4 of the remaining command difference each20ms.
// No fixed8us crawl; keep normal50Hz servo pulses and mechanical travel limits.
const uint8_t SERVO_FOLLOW_NUM = 3, SERVO_FOLLOW_DEN = 4;
const uint8_t MAX_FORWARD = 255;
const uint16_t POWER_ON_STABLE_MS = 500;
const uint32_t RX_TIMEOUT_US = 120000UL;
const uint16_t RX_MIN_US = 700, RX_MAX_US = 2300;
const uint16_t ADC_REFERENCE_MV = 5000; // Set to actual measured Nano 5V voltage.
const uint16_t BATTERY_CUTOFF_MV = 6400;

// Actual Nano logical pins; corrected RevL sockets preserve these nets.
const uint8_t RX1 = 2, FAULT = 4, IN1 = 5, IN2 = 6;
const uint8_t SLEEP = 8, SERVO_L = 9, SERVO_R = 10, LED = 11;
const uint8_t ENCODER_A = 7, ENCODER_B = 12, DISPLAY_CLK = A4, DISPLAY_DIO = A5;

static_assert(TEST_MODE == 0 || TEST_MODE == 1, "TEST_MODE must be 0 or 1");
static_assert(ENCODER_PULSES_NUM <= 1000000UL && ENCODER_TURNS_DEN > 0 && ENCODER_TURNS_DEN <= 1000UL, "Encoder ratio invalid");
static_assert(ENCODER_CALIBRATION_MODE == 0 || ENCODER_CALIBRATION_MODE == 1, "Calibration mode must be 0 or 1");
static_assert(TARGET_RPM == 600 && RPM_BAND_LOW == 570 && RPM_BAND_HIGH == 630, "Competition speed settings invalid");
static_assert(OVERSPEED_CUT_RPM > TARGET_RPM && OVERSPEED_CUT_RPM < RPM_BAND_HIGH, "Overspeed margin invalid");
static_assert(STARTUP_VERIFY_PWM <= STARTUP_PWM_MAX && STARTUP_HARD_MS >= ENCODER_VERIFY_MS, "Startup limits invalid");
static_assert(SERVO_FOLLOW_NUM > 0 && SERVO_FOLLOW_NUM <= SERVO_FOLLOW_DEN, "Servo follow ratio invalid");
static_assert(STEER_MIN < STEER_CENTER - STEER_DEAD && STEER_MAX > STEER_CENTER + STEER_DEAD, "Steer calibration invalid");
static_assert(LEFT_MIN >= 500 && LEFT_MIN < LEFT_CENTER && LEFT_CENTER < LEFT_MAX && LEFT_MAX <= 2500, "Left servo range invalid");
static_assert(RIGHT_MIN >= 500 && RIGHT_MIN < RIGHT_CENTER && RIGHT_CENTER < RIGHT_MAX && RIGHT_MAX <= 2500, "Right servo range invalid");

struct RxChannel {
  volatile uint32_t rise;
  volatile uint32_t lastGood;
  volatile uint16_t width;
  volatile uint8_t count;
  volatile bool high;
};
RxChannel rx1 = {0, 0, 0, 0, false};
Servo leftServo, rightServo;
volatile bool faultLatched = false;
volatile bool watchFault = false;
bool driverAwake = false;
bool armed = false, startTiming = false, batteryLatched = false, restartLocked = false;
bool good1 = false;
uint16_t pulse1 = 0, leftUs = LEFT_CENTER, rightUs = RIGHT_CENTER;
uint16_t batteryMv = 0;
int16_t appliedPWM = 0, drivenPWM = 0;
uint8_t state = 0; // 0 missingCH1;1 prestart;3 run;4 driver;5 battery;6 test;7 encoder;8 overspeed;9 lostRC latch;10 speed band
uint32_t startSince = 0, wakeSince = 0;
uint32_t lastServo = 0, lastBattery = 0, lastLog = 0, lowSince = 0;
bool lowTiming = false, batterySampled = false;
volatile uint32_t encoderCount = 0, encoderLastUs = 0, encoderPeriodUs = 0;
volatile uint32_t encoderPeriods[8] = {}, encoderPeriodSumUs = 0;
volatile uint8_t encoderPeriodCount = 0, encoderPeriodIndex = 0;
volatile bool encoderSeen = false;
volatile uint8_t encoderPortPrevious = 0;
uint32_t encoderTotal = 0, lastEncoderCalcUs = 0, encoderAgeUs = 0;
uint32_t measuredRPM = 0, lastEncoderLog = 0, lastDisplay = 0;
bool rpmHasEstimate = false, rpmFresh = false, displayOK = false;
bool rpmControlFresh = false, motionStarted = false, motionQualified = false, atCapacity = false;
bool startupVerifyTiming = false;
uint32_t startupVerifySince = 0;
uint8_t encoderFaultReason = 0; // 1 missing calibration,2 no startup motion,3 lost running pulses.
uint8_t speedFault = 0; // 1 encoder/calibration;2 overspeed;3 cannot hold tolerance.
uint16_t desiredRPM = 0, referenceRPM = 0;
uint32_t referenceQ8 = 0, motionStartedAt = 0, motionStartCount = 0, lastControl = 0, lastSpeedLog = 0;
int32_t speedIntegralQ8 = 0;
bool speedWithin5 = false, bandGoodTiming = false, bandBadTiming = false;
uint32_t bandGoodSince = 0, bandBadSince = 0;
const uint8_t DIGIT_SEGMENTS[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};

// Explicit declarations prevent Arduino auto-prototypes from obscuring types.
void captureEdge(RxChannel &rx, bool level, uint32_t now);
void rx1Changed();
void disableDriver();
void stopMotor(uint32_t now);
bool driverHasFault(uint32_t nowUs);
int16_t normalized(uint16_t pulse, uint16_t lo, uint16_t center, uint16_t hi, uint16_t dead);
void resetSpeedLoop(uint32_t now);
int16_t regulatedMotor(uint32_t now);
void outputMotor(int16_t command, uint32_t nowUs);
uint16_t rudderPulse(int16_t command, uint16_t lo, uint16_t center, uint16_t hi, bool reverse);
uint16_t moveToward(uint16_t current, uint16_t target);
void updateRudders(int16_t steering, uint32_t now);
void updateBattery(uint32_t now);
void updateLED(uint32_t now);
void report(uint32_t now);
uint32_t rpmFromPulses(uint32_t pulses, uint32_t intervalUs);
void updateEncoder(uint32_t nowUs);
void displayData(bool high);
void displayStart();
void displayStop();
bool displayByte(uint8_t value);
bool displayCommand(uint8_t value);
void updateDisplay(uint32_t now);
void reportEncoder(uint32_t now);
void reportSpeed(uint32_t now);
void checkSpeedBand(uint32_t now);

void captureEdge(RxChannel &rx, bool level, uint32_t now) {
  if (level) { rx.rise = now; rx.high = true; return; }
  if (!rx.high) return;
  rx.high = false;
  const uint32_t width = now - rx.rise;
  if (width < RX_MIN_US || width > RX_MAX_US) { rx.count = 0; return; }
  if (now - rx.lastGood >= RX_TIMEOUT_US) rx.count = 0;
  rx.width = static_cast<uint16_t>(width);
  rx.lastGood = now;
  if (rx.count < 3) ++rx.count;
}

void rx1Changed() { captureEdge(rx1, (PIND & _BV(PD2)) != 0, micros()); }

ISR(PCINT2_vect) {
  const uint8_t port = PIND;
  if (watchFault && !(port & _BV(PD4))) {
    PORTB &= ~_BV(PB0); // D8 LOW immediately, even between main-loop iterations.
    faultLatched = true;
  }
  const uint8_t changed = port ^ encoderPortPrevious;
  encoderPortPrevious = port;
  if ((changed & _BV(PD7)) && (port & _BV(PD7))) {
    const uint32_t now = micros(), elapsed = now - encoderLastUs;
    if (!encoderSeen || elapsed >= ENCODER_MIN_EDGE_US) {
      if (!encoderSeen || elapsed >= ENCODER_STOP_TIMEOUT_US) {
        encoderPeriodSumUs = 0; encoderPeriodCount = 0; encoderPeriodIndex = 0;
      } else {
        encoderPeriodUs = elapsed;
        if (encoderPeriodCount == 8) encoderPeriodSumUs -= encoderPeriods[encoderPeriodIndex];
        else ++encoderPeriodCount;
        encoderPeriods[encoderPeriodIndex] = elapsed;
        encoderPeriodSumUs += elapsed;
        encoderPeriodIndex = (encoderPeriodIndex + 1) & 7;
      }
      encoderSeen = true;
      encoderLastUs = now;
      ++encoderCount;
    }
  }
}

void disableDriver() {
  watchFault = false;
  digitalWrite(SLEEP, LOW);
  driverAwake = false;
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  drivenPWM = 0;
}

void stopMotor(uint32_t now) {
  appliedPWM = 0;
  disableDriver();
  resetSpeedLoop(now);
}

bool driverHasFault(uint32_t nowUs) {
  // Allow the specified wakeup interval before polling a pre-existing low.
  if (driverAwake && nowUs - wakeSince >= 2000) {
    watchFault = true;
    if (digitalRead(FAULT) == LOW) faultLatched = true;
  }
  return faultLatched;
}

int16_t normalized(uint16_t pulse, uint16_t lo, uint16_t center, uint16_t hi, uint16_t dead) {
  const int16_t p = constrain(static_cast<int16_t>(pulse), static_cast<int16_t>(lo), static_cast<int16_t>(hi));
  const int16_t upper = center + dead, lower = center - dead;
  if (p > upper) return static_cast<int32_t>(p - upper) * 1000 / (hi - upper);
  if (p < lower) return -static_cast<int32_t>(lower - p) * 1000 / (lower - lo);
  return 0;
}

void resetSpeedLoop(uint32_t now) {
  desiredRPM = 0; referenceRPM = 0; referenceQ8 = 0; speedIntegralQ8 = 0;
  motionStarted = false; motionQualified = false; atCapacity = false;
  startupVerifyTiming = false;
  lastControl = now;
  speedWithin5 = false; bandGoodTiming = false; bandBadTiming = false;
}

int16_t regulatedMotor(uint32_t now) {
  if (!MOTOR_ENABLED) { stopMotor(now); return 0; }
  if (!ENCODER_PULSES_NUM) { encoderFaultReason = 1; speedFault = 1; stopMotor(now); return 0; }
  desiredRPM = TARGET_RPM;
  if (motionStarted) {
    if (encoderTotal - motionStartCount >= 4 && rpmControlFresh) motionQualified = true;
    if (motionQualified && !rpmControlFresh) {
      encoderFaultReason = 3;
      speedFault = 1; stopMotor(now); return 0;
    }
    const bool startupExpired = now - motionStartedAt >= STARTUP_HARD_MS ||
      (startupVerifyTiming && now - startupVerifySince >= ENCODER_VERIFY_MS);
    if (!motionQualified && startupExpired) {
      encoderFaultReason = 2;
      speedFault = 1; stopMotor(now); return 0;
    }
  }
  if (rpmControlFresh && measuredRPM >= OVERSPEED_CUT_RPM) {
    speedFault = 2; stopMotor(now); return 0;
  }
  if (now - lastControl < 20) return appliedPWM;
  lastControl = now;
  const uint32_t desiredQ8 = static_cast<uint32_t>(desiredRPM) * 256UL;
  const uint16_t accelStepQ8 = static_cast<uint32_t>(SPEED_ACCEL_RPM_PER_SEC) * 256UL / 50UL;
  if (referenceQ8 < desiredQ8) referenceQ8 += min(static_cast<uint32_t>(accelStepQ8), desiredQ8 - referenceQ8);
  else referenceQ8 = desiredQ8; // Fixed power-switch target; no throttle input.
  referenceRPM = (referenceQ8 + 128UL) / 256UL;
  const int16_t feedback = rpmControlFresh ? min(measuredRPM, 2000UL) : 0;
  const int16_t error = static_cast<int16_t>(referenceRPM) - feedback;
  const uint8_t limit = motionQualified ? MAX_FORWARD : STARTUP_PWM_MAX;
  const int32_t base = static_cast<int32_t>(referenceRPM) * SPEED_FF_Q8 + static_cast<int32_t>(error) * SPEED_KP_Q8;
  const int32_t nextIntegral = constrain(speedIntegralQ8 + static_cast<int32_t>(error) * SPEED_KI_STEP_Q8, -20480L, 65280L);
  const int32_t trial = base + nextIntegral;
  // Conditional integration avoids accumulating unreachable output demand.
  if (!((referenceQ8 < desiredQ8 && error > 0) ||
        (trial > static_cast<int32_t>(limit) * 256 && error > 0) || (trial < 0 && error < 0)))
    speedIntegralQ8 = nextIntegral;
  int16_t duty = constrain((base + speedIntegralQ8 + 128L) / 256L, 0L, static_cast<int32_t>(limit));
  int16_t previous = abs(appliedPWM);
  // Above 600, reduce drive now; do not disguise overspeed on the display.
  if (rpmControlFresh && measuredRPM >= TARGET_RPM) duty = min(duty, previous);
  if (duty > previous + 6) duty = previous + 6; // Gradual rise, <=6/255 per20ms.
  atCapacity = duty >= limit && motionQualified && error > 5; // SAT=1: little/no drive headroom.
  appliedPWM = duty;
  if (appliedPWM) {
    if (!motionStarted) { motionStarted = true; motionStartedAt = now; motionStartCount = encoderTotal; }
    if (!motionQualified && !startupVerifyTiming && appliedPWM >= STARTUP_VERIFY_PWM) {
      startupVerifyTiming = true; startupVerifySince = now;
    }
  }
  return appliedPWM;
}

void outputMotor(int16_t command, uint32_t nowUs) {
  if (!command || !MOTOR_ENABLED || driverHasFault(nowUs)) { disableDriver(); return; }
  if (!driverAwake) {
    analogWrite(IN1, 0); analogWrite(IN2, 0);
    watchFault = false; // nFAULT may be unsettled during driver wakeup.
    digitalWrite(SLEEP, HIGH);
    driverAwake = true;
    wakeSince = nowUs;
    return;
  }
  if (nowUs - wakeSince < 2000) return;
  if (driverHasFault(nowUs)) { disableDriver(); return; }
  const int16_t actual = MOTOR_REVERSED ? -command : command;
  if (actual > 0) { analogWrite(IN2, 0); analogWrite(IN1, actual); }
  else { analogWrite(IN1, 0); analogWrite(IN2, -actual); }
  drivenPWM = command;
}

uint16_t rudderPulse(int16_t command, uint16_t lo, uint16_t center, uint16_t hi, bool reverse) {
  if (reverse) command = -command;
  const int16_t span = command >= 0 ? hi - center : center - lo;
  return center + static_cast<int32_t>(command) * span / 1000;
}

uint16_t moveToward(uint16_t current, uint16_t target) {
  const int16_t difference = static_cast<int16_t>(target) - current;
  if (abs(difference) <= 2) return target;
  return current + static_cast<int32_t>(difference) * SERVO_FOLLOW_NUM / SERVO_FOLLOW_DEN;
}

void updateRudders(int16_t steering, uint32_t now) {
  if (now - lastServo < 20) return;
  lastServo = now;
  uint16_t targetL, targetR;
#if TEST_MODE == 1
  (void)steering;
  const uint16_t targets[4] = {1500, 1400, 1500, 1600};
  targetL = targetR = targets[(now / 1200UL) % 4];
#else
  targetL = rudderPulse(steering, LEFT_MIN, LEFT_CENTER, LEFT_MAX, LEFT_REVERSED);
  targetR = rudderPulse(steering, RIGHT_MIN, RIGHT_CENTER, RIGHT_MAX, RIGHT_REVERSED);
#endif
  leftUs = moveToward(leftUs, targetL);
  rightUs = moveToward(rightUs, targetR);
  leftServo.writeMicroseconds(leftUs);
  rightServo.writeMicroseconds(rightUs);
}

void updateBattery(uint32_t now) {
  if (batterySampled && now - lastBattery < 50) return;
  lastBattery = now;
  const uint16_t mv = (static_cast<uint32_t>(analogRead(A0)) * ADC_REFERENCE_MV * 43UL + 5115UL) / 10230UL;
  if (!batterySampled) { batteryMv = mv; batterySampled = true; }
  else batteryMv = static_cast<int32_t>(batteryMv) + (static_cast<int32_t>(mv) - batteryMv) / 8;
#if ENABLE_BATTERY_CUTOFF
  if (batteryMv < BATTERY_CUTOFF_MV) {
    if (!lowTiming) { lowTiming = true; lowSince = now; }
    if (now - lowSince >= 1500) batteryLatched = true;
  } else lowTiming = false;
#endif
}

void updateLED(uint32_t now) {
  const uint16_t p = now % 1500UL;
  bool on;
  if (state == 4 || state == 7 || state == 8 || state == 9 || state == 10) on = now % 200 < 100;
  else if (state == 5 || state == 6) on = p < 100 || (p >= 200 && p < 300) || (p >= 400 && p < 500);
  else if (!good1) on = now % 1000 < 100; // One flash: steering missing.
  else if (!armed) on = now % 1000 < 500;
  else on = drivenPWM != 0 || p < 80;
  digitalWrite(LED, on ? HIGH : LOW);
}

void report(uint32_t now) {
  if (now - lastLog < 500 || Serial.availableForWrite() < 60) return;
  lastLog = now;
  char text[60];
  const int n = snprintf(text, sizeof(text), "S=%u CH1=%u OK=%u RUN=%u BAT=%u PWM=%d\n",
      state, pulse1, good1 ? 1 : 0, armed ? 1 : 0, batteryMv, drivenPWM);
  if (n > 0 && n < static_cast<int>(sizeof(text))) Serial.write(reinterpret_cast<const uint8_t*>(text), n);
}

// The ratio NUM / DEN is A-channel rising edges per PROPELLER revolution.
// Direction is deliberately not inferred: displayed RPM is the magnitude.
uint32_t rpmFromPulses(uint32_t pulses, uint32_t intervalUs) {
  if (!ENCODER_PULSES_NUM || !intervalUs) return 0;
  const uint64_t numerator = static_cast<uint64_t>(pulses) * 60000000ULL * ENCODER_TURNS_DEN;
  const uint64_t denominator = static_cast<uint64_t>(intervalUs) * ENCODER_PULSES_NUM;
  const uint64_t value = (numerator + denominator / 2) / denominator;
  return value > 99999UL ? 99999UL : static_cast<uint32_t>(value);
}

void updateEncoder(uint32_t nowUs) {
  if (nowUs - lastEncoderCalcUs < 5000UL) return;
  uint32_t total, last, sum;
  uint8_t periods;
  bool seen;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    total = encoderCount; last = encoderLastUs;
    sum = encoderPeriodSumUs; periods = encoderPeriodCount; seen = encoderSeen;
  }
  encoderTotal = total;
  // Sample time MUST be read after the ISR-owned timestamp snapshot.
  nowUs = micros();
  lastEncoderCalcUs = nowUs;
  encoderAgeUs = nowUs - last;
  rpmFresh = seen && encoderAgeUs < ENCODER_STOP_TIMEOUT_US;
  rpmControlFresh = seen && periods >= 2 && encoderAgeUs < static_cast<uint32_t>(ENCODER_LOSS_MS) * 1000UL;
  rpmHasEstimate = ENCODER_PULSES_NUM && rpmFresh && periods > 0 && sum;
  measuredRPM = rpmHasEstimate ? rpmFromPulses(periods, sum) : 0;
}

// TM1637 transactions leave interrupts enabled for RC, servos and encoder.
// DIO uses release/LOW so display ACK cannot contend with a driven HIGH.
void displayData(bool high) {
  if (high) pinMode(DISPLAY_DIO, INPUT_PULLUP);
  else { digitalWrite(DISPLAY_DIO, LOW); pinMode(DISPLAY_DIO, OUTPUT); }
}

void displayStart() {
  displayData(true); digitalWrite(DISPLAY_CLK, HIGH); delayMicroseconds(20);
  displayData(false); delayMicroseconds(20); digitalWrite(DISPLAY_CLK, LOW);
}

void displayStop() {
  digitalWrite(DISPLAY_CLK, LOW); displayData(false); delayMicroseconds(20);
  digitalWrite(DISPLAY_CLK, HIGH); delayMicroseconds(20);
  displayData(true); delayMicroseconds(20);
}

bool displayByte(uint8_t value) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    digitalWrite(DISPLAY_CLK, LOW); displayData(value & 1);
    delayMicroseconds(20); digitalWrite(DISPLAY_CLK, HIGH); delayMicroseconds(20);
    value >>= 1;
  }
  digitalWrite(DISPLAY_CLK, LOW); displayData(true); delayMicroseconds(20);
  digitalWrite(DISPLAY_CLK, HIGH); delayMicroseconds(20);
  const bool ok = digitalRead(DISPLAY_DIO) == LOW;
  digitalWrite(DISPLAY_CLK, LOW); delayMicroseconds(20);
  return ok;
}

bool displayCommand(uint8_t value) {
  displayStart(); const bool ok = displayByte(value); displayStop(); return ok;
}

void updateDisplay(uint32_t now) {
  if (now - lastDisplay < 250) return;
  lastDisplay = now;
  uint8_t digits[4] = {0x40, 0x40, 0x40, 0x40}; // ----: calibration unknown/overflow.
  const uint32_t displayValue = ENCODER_CALIBRATION_MODE ? encoderTotal : measuredRPM;
  const bool canDisplay = ENCODER_CALIBRATION_MODE ||
    (ENCODER_PULSES_NUM && (!rpmFresh || rpmHasEstimate));
  if (canDisplay && displayValue <= 9999UL) {
    uint16_t value = displayValue;
    for (int8_t i = 3; i >= 0; --i) { digits[i] = DIGIT_SEGMENTS[value % 10]; value /= 10; }
  }
  if (!ENCODER_CALIBRATION_MODE && speedFault) {
    digits[0] = 0x79; digits[1] = 0x3F; digits[2] = 0x3F;
    digits[3] = DIGIT_SEGMENTS[speedFault]; // E001 encoder;E002 overspeed;E003 band.
  } else if (!ENCODER_CALIBRATION_MODE && restartLocked && !faultLatched && !batteryLatched) {
    digits[0] = 0x79; digits[1] = 0x3F; digits[2] = 0x3F; digits[3] = DIGIT_SEGMENTS[4]; // E004 lost steering.
  }
  // TEMPORARY diagnostic: alternate measuredRPM and actual commanded duty.
  // P255 = full command, not a measurement of delivered motor voltage/current.
  // Fault displays above retain priority; no motor control changes here.
  if (!ENCODER_CALIBRATION_MODE && armed && !speedFault && !restartLocked &&
      !faultLatched && !batteryLatched && ((now / 2000UL) & 1)) {
    const uint16_t pwm = abs(drivenPWM);
    digits[0] = 0x73; // P
    digits[1] = DIGIT_SEGMENTS[(pwm / 100) % 10];
    digits[2] = DIGIT_SEGMENTS[(pwm / 10) % 10];
    digits[3] = DIGIT_SEGMENTS[pwm % 10];
  }
  bool ok = displayCommand(0x40);
  displayStart(); ok &= displayByte(0xC0);
  for (uint8_t i = 0; i < 4; ++i) ok &= displayByte(digits[i]);
  displayStop(); ok &= displayCommand(0x88 | DISPLAY_BRIGHTNESS);
  displayOK = ok;
}

void reportEncoder(uint32_t now) {
  if (now - lastEncoderLog < 1000 || Serial.availableForWrite() < 60) return;
  lastEncoderLog = now;
  char line[60];
  const int n = snprintf(line, sizeof(line), "ENC CNT=%lu RPM=%lu CAL=%u FRESH=%u DISP=%u\n",
      static_cast<unsigned long>(encoderTotal), static_cast<unsigned long>(measuredRPM),
      ENCODER_PULSES_NUM ? 1 : 0, rpmFresh ? 1 : 0, displayOK ? 1 : 0);
  if (n > 0 && n < static_cast<int>(sizeof(line))) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
}

void reportSpeed(uint32_t now) {
  if (now - lastSpeedLog < 1000 || Serial.availableForWrite() < 60) return;
  lastSpeedLog = now;
  char line[60];
  const int n = snprintf(line, sizeof(line), "SPD T=%u REF=%u F=%u SAT=%u OK5=%u WHY=%u\n", desiredRPM, referenceRPM, speedFault, atCapacity ? 1 : 0, speedWithin5 ? 1 : 0, encoderFaultReason);
  if (n > 0 && n < static_cast<int>(sizeof(line))) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
}

void checkSpeedBand(uint32_t now) {
  if (!armed || referenceRPM < TARGET_RPM || !rpmControlFresh) {
    speedWithin5 = false; bandGoodTiming = false; bandBadTiming = false; return;
  }
  if (measuredRPM >= RPM_BAND_LOW && measuredRPM <= RPM_BAND_HIGH) {
    bandBadTiming = false;
    if (!bandGoodTiming) { bandGoodTiming = true; bandGoodSince = now; }
    speedWithin5 = now - bandGoodSince >= SPEED_BAND_HOLD_MS;
  } else {
    speedWithin5 = false; bandGoodTiming = false;
    if (!bandBadTiming) { bandBadTiming = true; bandBadSince = now; }
    if (now - bandBadSince >= SPEED_BAND_FAIL_MS) speedFault = 3;
  }
}

void setup() {
  digitalWrite(SLEEP, LOW); pinMode(SLEEP, OUTPUT);
  digitalWrite(IN1, LOW); pinMode(IN1, OUTPUT);
  digitalWrite(IN2, LOW); pinMode(IN2, OUTPUT);
  pinMode(LED, OUTPUT); pinMode(FAULT, INPUT_PULLUP);
  pinMode(RX1, INPUT); // Existing board pulldown. CH2 is not used.
  disableDriver();
  if (!ENCODER_CALIBRATION_MODE) {
    leftServo.attach(SERVO_L, 500, 2500);
    rightServo.attach(SERVO_R, 500, 2500);
    leftServo.writeMicroseconds(leftUs); rightServo.writeMicroseconds(rightUs);
  } else {
    digitalWrite(SERVO_L, LOW); pinMode(SERVO_L, OUTPUT);
    digitalWrite(SERVO_R, LOW); pinMode(SERVO_R, OUTPUT);
  }
  pinMode(ENCODER_A, INPUT_PULLUP); pinMode(ENCODER_B, INPUT);
  digitalWrite(DISPLAY_CLK, HIGH); pinMode(DISPLAY_CLK, OUTPUT); displayData(true);
  attachInterrupt(digitalPinToInterrupt(RX1), rx1Changed, CHANGE);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderPortPrevious = PIND;
    PCMSK2 |= _BV(PD4) | _BV(PD7); PCIFR = _BV(PCIF2); PCICR |= _BV(PCIE2);
  }
  analogReference(DEFAULT);
  Serial.begin(115200);
  Serial.println(F("Boat V5.3 DIAG RPM/PWM; AUTO600; CH1 only"));
  if (!ENCODER_PULSES_NUM) Serial.println(F("RPM UNCALIBRATED: display ----, raw CNT available"));
}

void loop() {
  uint32_t time1;
  uint8_t count1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pulse1 = rx1.width; time1 = rx1.lastGood; count1 = rx1.count;
  }
  const uint32_t nowUs = micros(), now = millis();
  good1 = count1 >= 3 && nowUs - time1 < RX_TIMEOUT_US;
  updateEncoder(micros());
  updateBattery(now);
  driverHasFault(nowUs);
  int16_t steering = good1 ? normalized(pulse1, STEER_MIN, STEER_CENTER, STEER_MAX, STEER_DEAD) : 0;
  if (STEERING_REVERSED) steering = -steering;
  if (!ENCODER_CALIBRATION_MODE) updateRudders(steering, now);

#if TEST_MODE == 1 || ENCODER_CALIBRATION_MODE == 1
  armed = false; startTiming = false; state = 6; stopMotor(now);
#else
  if (faultLatched || batteryLatched || speedFault || restartLocked) {
    restartLocked = true; armed = false; startTiming = false;
    state = faultLatched ? 4 : batteryLatched ? 5 : speedFault == 1 ? 7 : speedFault == 2 ? 8 : speedFault == 3 ? 10 : 9;
    stopMotor(now);
  } else if (!good1) {
    if (armed) restartLocked = true;
    armed = false; startTiming = false; state = restartLocked ? 9 : 0; stopMotor(now);
  } else {
    if (!armed) {
      if (!startTiming) { startTiming = true; startSince = now; }
      else if (now - startSince >= POWER_ON_STABLE_MS) armed = true;
    }
    state = armed ? 3 : 1;
    if (!armed) stopMotor(now);
    else {
      const int16_t command = regulatedMotor(now);
      outputMotor(command, micros());
      checkSpeedBand(millis());
      if (speedFault) {
        restartLocked = true; armed = false; state = speedFault == 1 ? 7 : speedFault == 2 ? 8 : 10; stopMotor(now);
      }
    }
  }
  if (driverHasFault(micros())) { restartLocked = true; armed = false; state = 4; stopMotor(now); }
#endif
  updateLED(now);
  report(now);
  updateDisplay(now);
  reportEncoder(millis());
  reportSpeed(millis());
}
