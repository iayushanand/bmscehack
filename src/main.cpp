#include <Arduino.h>
#include <BluetoothSerial.h>
#include <stdint.h>

/*
 * ESP32 + L298N dual BO motor control via Bluetooth Classic (SPP).
 * Hold-to-run command protocol (send repeatedly while button is pressed):
 *   'F' -> robot forward
 *   'B' -> robot backward
 *   'L' -> robot left
 *   'R' -> robot right
 *   'S' -> stop immediately
 */

/* Left motor (L298N channel A) */
static constexpr uint8_t MOTOR_LEFT_IN1_PIN = 26U;
static constexpr uint8_t MOTOR_LEFT_IN2_PIN = 27U;
static constexpr uint8_t MOTOR_LEFT_EN_PIN = 25U;

/* Right motor (L298N channel B) */
static constexpr uint8_t MOTOR_RIGHT_IN1_PIN = 14U;
static constexpr uint8_t MOTOR_RIGHT_IN2_PIN = 12U;
static constexpr uint8_t MOTOR_RIGHT_EN_PIN = 33U;

static constexpr uint8_t PWM_CH_LEFT = 0U;
static constexpr uint8_t PWM_CH_RIGHT = 1U;
static constexpr uint32_t PWM_FREQ_HZ = 1000UL;
static constexpr uint8_t PWM_RES_BITS = 8U;
static constexpr uint8_t MOTOR_SPEED_DUTY = 100U; /* 0..255 */
static constexpr uint32_t COMMAND_HOLD_TIMEOUT_MS = 250UL;

static const char *BT_DEVICE_NAME = "BURNT_TOASTER";

static BluetoothSerial g_btSerial;
static volatile uint32_t g_lastMotionCommandMs = 0UL;

static void setLeftMotor(const bool forward, const uint8_t duty)
{
  digitalWrite(MOTOR_LEFT_IN1_PIN, forward ? HIGH : LOW);
  digitalWrite(MOTOR_LEFT_IN2_PIN, forward ? LOW : HIGH);
  ledcWrite(PWM_CH_LEFT, duty);
}

static void setRightMotor(const bool forward, const uint8_t duty)
{
  digitalWrite(MOTOR_RIGHT_IN1_PIN, forward ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_IN2_PIN, forward ? LOW : HIGH);
  ledcWrite(PWM_CH_RIGHT, duty);
}

static void robotStop(void)
{
  digitalWrite(MOTOR_LEFT_IN1_PIN, LOW);
  digitalWrite(MOTOR_LEFT_IN2_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_IN1_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_IN2_PIN, LOW);
  ledcWrite(PWM_CH_LEFT, 0U);
  ledcWrite(PWM_CH_RIGHT, 0U);
}

static void robotForward(void)
{
  setLeftMotor(true, MOTOR_SPEED_DUTY);
  setRightMotor(true, MOTOR_SPEED_DUTY);
}

static void robotBackward(void)
{
  setLeftMotor(false, MOTOR_SPEED_DUTY);
  setRightMotor(false, MOTOR_SPEED_DUTY);
}

static void robotLeft(void)
{
  /* Pivot left: left wheel backward, right wheel forward */
  setLeftMotor(false, MOTOR_SPEED_DUTY);
  setRightMotor(true, MOTOR_SPEED_DUTY);
}

static void robotRight(void)
{
  /* Pivot right: left wheel forward, right wheel backward */
  setLeftMotor(true, MOTOR_SPEED_DUTY);
  setRightMotor(false, MOTOR_SPEED_DUTY);
}

static void handleCommand(const char command)
{
  switch (command)
  {
    case 'F':
    case 'f':
      robotForward();
      g_lastMotionCommandMs = millis();
      break;

    case 'B':
    case 'b':
      robotBackward();
      g_lastMotionCommandMs = millis();
      break;

    case 'L':
    case 'l':
      robotLeft();
      g_lastMotionCommandMs = millis();
      break;

    case 'R':
    case 'r':
      robotRight();
      g_lastMotionCommandMs = millis();
      break;

    case 'S':
    case 's':
      robotStop();
      g_lastMotionCommandMs = 0UL;
      break;

    default:
      /* Ignore unsupported commands */
      break;
  }
}

void setup()
{
  pinMode(MOTOR_LEFT_IN1_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_IN2_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_EN_PIN, OUTPUT);

  pinMode(MOTOR_RIGHT_IN1_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_EN_PIN, OUTPUT);

  ledcSetup(PWM_CH_LEFT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MOTOR_LEFT_EN_PIN, PWM_CH_LEFT);
  ledcAttachPin(MOTOR_RIGHT_EN_PIN, PWM_CH_RIGHT);

  robotStop();

  (void)g_btSerial.begin(BT_DEVICE_NAME);
  g_btSerial.println("Bluetooth dual-motor hold-to-run ready");
  g_btSerial.println("Hold F/B/L/R. Release to auto-stop.");
  g_btSerial.println("Send S to stop immediately.");

  g_lastMotionCommandMs = 0UL;
}

void loop()
{
  while (g_btSerial.available() > 0)
  {
    const char cmd = static_cast<char>(g_btSerial.read());
    handleCommand(cmd);
  }

  if (g_lastMotionCommandMs > 0UL)
  {
    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - g_lastMotionCommandMs;

    if (elapsedMs > COMMAND_HOLD_TIMEOUT_MS)
    {
      robotStop();
      g_lastMotionCommandMs = 0UL;
    }
  }

  delay(10U);
}
