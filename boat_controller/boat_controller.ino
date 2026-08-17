/*
 * 单桨水面航行器——油门接收与有刷电机控制
 *
 * 硬件：
 *   Arduino Nano (ATmega328P, 5 V / 16 MHz)
 *   普通 2.4 GHz 遥控接收机
 *   L298N 双 H 桥模块（这里只使用 A 路）
 *   6 V / 600 r/min 25-370 减速电机
 *
 * 接线：
 *   接收机 CH2 信号 -> Nano D2
 *   L298N ENA       -> Nano D5（PWM；拔掉 ENA 跳帽）
 *   L298N IN1       -> Nano D7
 *   L298N IN2       -> Nano D8
 *   BEC 5 V         -> Nano 5V、接收机 VCC、L298N 5V logic
 *   所有 GND 共地
 *
 * 重要：拔掉 L298N 板上的 5V-EN 跳帽，由独立 5 V BEC 给逻辑侧供电。
 * 电池 2S 直接给 L298N 的 VM/12V 电机电源端，不能接到 Nano 5V。
 */

constexpr uint8_t RC_THROTTLE_PIN = 2;
constexpr uint8_t MOTOR_PWM_PIN   = 5;
constexpr uint8_t MOTOR_IN1_PIN   = 7;
constexpr uint8_t MOTOR_IN2_PIN   = 8;

constexpr uint16_t RC_MIN_US        = 1000;
constexpr uint16_t RC_NEUTRAL_US    = 1500;
constexpr uint16_t RC_MAX_US        = 2000;
constexpr uint16_t RC_DEADBAND_US   = 45;
constexpr uint32_t RC_TIMEOUT_US    = 30000;

// 175/255 = 68.6%。用于把 2S 满电时的最高电机转速留在 600 r/min 以下。
// 最终必须以实测转速为准；若实测超过 550 r/min，继续下调这个值。
constexpr uint8_t MAX_PWM = 175;

// 正反转切换前先停机，减少齿轮箱、电机和驱动板冲击。
constexpr uint16_t DIRECTION_CHANGE_STOP_MS = 180;

int8_t lastDirection = 0;

void stopMotor() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  lastDirection = 0;
}

void driveMotor(int8_t direction, uint8_t pwm) {
  if (direction == 0 || pwm == 0) {
    stopMotor();
    return;
  }

  if (lastDirection != 0 && direction != lastDirection) {
    stopMotor();
    delay(DIRECTION_CHANGE_STOP_MS);
  }

  digitalWrite(MOTOR_IN1_PIN, direction > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN2_PIN, direction > 0 ? LOW : HIGH);
  analogWrite(MOTOR_PWM_PIN, pwm);
  lastDirection = direction;
}

void setup() {
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  stopMotor();
}

void loop() {
  const uint32_t pulseUs = pulseIn(RC_THROTTLE_PIN, HIGH, RC_TIMEOUT_US);

  // 接收机无信号、断电或脉宽异常时立即停机。
  if (pulseUs < 850 || pulseUs > 2150) {
    stopMotor();
    return;
  }

  const int32_t offset = static_cast<int32_t>(pulseUs) - RC_NEUTRAL_US;
  if (abs(offset) <= RC_DEADBAND_US) {
    stopMotor();
    return;
  }

  int8_t direction;
  uint16_t magnitudeUs;
  if (offset > 0) {
    direction = 1;
    magnitudeUs = static_cast<uint16_t>(offset - RC_DEADBAND_US);
  } else {
    direction = -1;
    magnitudeUs = static_cast<uint16_t>(-offset - RC_DEADBAND_US);
  }

  const uint16_t usableSpanUs = (RC_MAX_US - RC_NEUTRAL_US) - RC_DEADBAND_US;
  magnitudeUs = constrain(magnitudeUs, 0, usableSpanUs);
  const uint8_t pwm = static_cast<uint8_t>(map(magnitudeUs, 0, usableSpanUs, 0, MAX_PWM));
  driveMotor(direction, pwm);
}
