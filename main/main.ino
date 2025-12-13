#include "config.h"
#include "motor.h"
#include <Arduino.h>

uint8_t readLineSensors();
int calculateLineError(uint8_t pattern, int previousError);
void lineTraceControl();
void resetLineTraceController();

/* =========================
 * グローバル変数
 * ========================= */
RobotState currentState = STOP;
RobotState previousState = STOP;
Motor motor(LINE_BASE_SPEED); // 初期速度をベーススピードに合わせる
float lineIntegral = 0.0f;
int linePreviousError = 0;
unsigned long lastLineUpdate = 0;

/* =========================
 * setup
 * ========================= */
void setup() {
  // ===== モータピン設定 =====
  pinMode(Motor_L1_F_PIN, OUTPUT);
  pinMode(Motor_L1_B_PIN, OUTPUT);
  pinMode(Motor_L2_F_PIN, OUTPUT);
  pinMode(Motor_L2_B_PIN, OUTPUT);
  pinMode(Motor_R1_F_PIN, OUTPUT);
  pinMode(Motor_R1_B_PIN, OUTPUT);
  pinMode(Motor_R2_F_PIN, OUTPUT);
  pinMode(Motor_R2_B_PIN, OUTPUT);

  // ===== PWM 初期化（ESP32 LEDC）=====
  int motorPins[] = {Motor_L1_F_PIN, Motor_L1_B_PIN, Motor_L2_F_PIN,
                     Motor_L2_B_PIN, Motor_R1_F_PIN, Motor_R1_B_PIN,
                     Motor_R2_F_PIN, Motor_R2_B_PIN};

  for (int i = 0; i < 8; i++) {
    ledcAttach(motorPins[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(motorPins[i], 0);
  }

  // ===== STBY ピン有効化 =====
  pinMode(CONTROL_PIN_2, OUTPUT);
  pinMode(CONTROL_PIN_15, OUTPUT);
  digitalWrite(CONTROL_PIN_2, HIGH);
  digitalWrite(CONTROL_PIN_15, HIGH);

  // ===== フォトリフレクタ =====
  for (int i = 0; i < 8; i++) {
    pinMode(Photoreflector_PIN[i], INPUT);
  }

  // ===== Serial =====
  Serial.begin(115200);
  Serial.println("ESP32 4WD Robot Control");
  Serial.println("Commands:");
  Serial.println("0/s: STOP");
  Serial.println("1/f: FORWARD");
  Serial.println("2/b: BACKWARD");
  Serial.println("3/l: LEFT");
  Serial.println("4/r: RIGHT");
  Serial.println("5/L: TURN LEFT");
  Serial.println("6/R: TURN RIGHT");
  Serial.println("7/t: LINE TRACE");

  // 初期停止
  motor.stop();
}

/* =========================
 * loop
 * ========================= */
void loop() {

  // ===== コマンド受付 =====
  if (Serial.available() > 0) {
    char command = Serial.read();

    // 改行は無視
    if (command == '\r' || command == '\n') {
      return;
    }

    switch (command) {
    case '0':
    case 's':
      currentState = STOP;
      break;

    case '1':
    case 'f':
      currentState = FORWARD;
      break;

    case '2':
    case 'b':
      currentState = BACKWARD;
      break;

    case '3':
    case 'l':
      currentState = LEFT;
      break;

    case '4':
    case 'r':
      currentState = RIGHT;
      break;

    case '5':
    case 'L':
      currentState = TURN_LEFT;
      break;

    case '6':
    case 'R':
      currentState = TURN_RIGHT;
      break;

    case '7':
    case 't':
    case 'T':
      currentState = LINE_TRACE;
      break;

    default:
      Serial.println("Invalid command");
      return;
    }
  }

  if (previousState != currentState) {
    if (currentState == LINE_TRACE) {
      resetLineTraceController();
    } else if (previousState == LINE_TRACE) {
      motor.stop();
    }
    previousState = currentState;
  }

  // ===== 状態に応じた動作 =====
  switch (currentState) {
  case STOP:
    motor.stop();
    break;

  case FORWARD:
    motor.forward();
    break;

  case BACKWARD:
    motor.back();
    break;

  case LEFT:
    motor.left();
    break;

  case RIGHT:
    motor.right();
    break;

  case TURN_LEFT:
    motor.turnLeft();
    break;

  case TURN_RIGHT:
    motor.turnRight();
    break;

  case LINE_TRACE:
    lineTraceControl();
    break;
  }
}

uint8_t readLineSensors() {
  uint8_t pattern = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(Photoreflector_PIN[i]);
    bool active =
        LINE_SENSOR_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
    if (active) {
      pattern |= (1 << i);
    }
  }
  return pattern;
}

int calculateLineError(uint8_t pattern, int previousError) {
  int weightedSum = 0;
  int activeCount = 0;
  for (int i = 0; i < 8; i++) {
    if (pattern & (1 << i)) {
      weightedSum += LINE_SENSOR_WEIGHTS[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    return previousError;
  }

  return weightedSum / activeCount;
}

void resetLineTraceController() {
  lineIntegral = 0.0f;
  linePreviousError = 0;
  lastLineUpdate = 0;
}

void lineTraceControl() {
  unsigned long now = millis();
  if (lastLineUpdate != 0 &&
      (now - lastLineUpdate) < LINE_CONTROL_INTERVAL_MS) {
    return;
  }

  float dt =
      (lastLineUpdate == 0)
          ? (LINE_CONTROL_INTERVAL_MS / 1000.0f)
          : (now - lastLineUpdate) / 1000.0f;
  lastLineUpdate = now;

  uint8_t pattern = readLineSensors();
  int error = calculateLineError(pattern, linePreviousError);

  float derivative =
      dt > 0 ? static_cast<float>(error - linePreviousError) / dt : 0.0f;
  lineIntegral += static_cast<float>(error) * dt;
  lineIntegral = constrain(lineIntegral, -LINE_INTEGRAL_LIMIT,
                           LINE_INTEGRAL_LIMIT);

  float correction =
      (LINE_KP * error) + (LINE_KI * lineIntegral) + (LINE_KD * derivative);
  int correctionValue = static_cast<int>(correction);

  int leftSpeed = LINE_BASE_SPEED - correctionValue;
  int rightSpeed = LINE_BASE_SPEED + correctionValue;
  leftSpeed = constrain(leftSpeed, -LINE_MAX_SPEED, LINE_MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -LINE_MAX_SPEED, LINE_MAX_SPEED);

  motor.drive(leftSpeed, rightSpeed);
  linePreviousError = error;
}
