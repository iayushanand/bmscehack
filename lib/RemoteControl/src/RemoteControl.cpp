#include "RemoteControl.h"

RemoteControl::RemoteControl(const Config &config)
    : m_config(config)
{
}

RemoteControl::RemoteControl()
    : m_config(Config())
{
}

void RemoteControl::begin()
{
  pinMode(m_config.pinLeftIn1, OUTPUT);
  pinMode(m_config.pinLeftIn2, OUTPUT);
  pinMode(m_config.pinLeftEn, OUTPUT);

  pinMode(m_config.pinRightIn1, OUTPUT);
  pinMode(m_config.pinRightIn2, OUTPUT);
  pinMode(m_config.pinRightEn, OUTPUT);

  ledcSetup(m_config.pwmChannelLeft, m_config.pwmFreqHz, m_config.pwmResolutionBits);
  ledcSetup(m_config.pwmChannelRight, m_config.pwmFreqHz, m_config.pwmResolutionBits);
  ledcAttachPin(m_config.pinLeftEn, m_config.pwmChannelLeft);
  ledcAttachPin(m_config.pinRightEn, m_config.pwmChannelRight);

  stop();

  (void)m_btSerial.begin(m_config.btDeviceName);
  m_btSerial.println("Bluetooth dual-motor hold-to-run ready");
  m_btSerial.println("Hold F/B/L/R. Release to auto-stop.");
  m_btSerial.println("Send S to stop immediately.");

  m_lastMotionCommandMs = 0UL;
  m_initialized = true;
}

void RemoteControl::begin(const char *btDeviceName, uint8_t motorSpeedDuty)
{
  m_config.btDeviceName = btDeviceName;
  m_config.motorSpeedDuty = motorSpeedDuty;
  begin();
}

void RemoteControl::update()
{
  while (m_btSerial.available() > 0)
  {
    const char cmd = static_cast<char>(m_btSerial.read());
    handleCommand(cmd);
  }

  if (m_lastMotionCommandMs > 0UL)
  {
    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - m_lastMotionCommandMs;
    if (elapsedMs > m_config.holdTimeoutMs)
    {
      stop();
      m_lastMotionCommandMs = 0UL;
    }
  }
}

void RemoteControl::handleCommand(const char command)
{
  switch (command)
  {
    case 'F':
    case 'f':
      forward();
      m_lastMotionCommandMs = millis();
      break;

    case 'B':
    case 'b':
      backward();
      m_lastMotionCommandMs = millis();
      break;

    case 'L':
    case 'l':
      left();
      m_lastMotionCommandMs = millis();
      break;

    case 'R':
    case 'r':
      right();
      m_lastMotionCommandMs = millis();
      break;

    case 'S':
    case 's':
      stop();
      m_lastMotionCommandMs = 0UL;
      break;

    default:
      // Ignore unsupported commands (including '\r', '\n')
      break;
  }
}

void RemoteControl::stop()
{
  digitalWrite(m_config.pinLeftIn1, LOW);
  digitalWrite(m_config.pinLeftIn2, LOW);
  digitalWrite(m_config.pinRightIn1, LOW);
  digitalWrite(m_config.pinRightIn2, LOW);
  ledcWrite(m_config.pwmChannelLeft, 0U);
  ledcWrite(m_config.pwmChannelRight, 0U);
}

void RemoteControl::forward()
{
  setLeftMotor(true, m_config.motorSpeedDuty);
  setRightMotor(true, m_config.motorSpeedDuty);
}

void RemoteControl::backward()
{
  setLeftMotor(false, m_config.motorSpeedDuty);
  setRightMotor(false, m_config.motorSpeedDuty);
}

void RemoteControl::left()
{
  // Pivot left: left wheel backward, right wheel forward
  setLeftMotor(false, m_config.motorSpeedDuty);
  setRightMotor(true, m_config.motorSpeedDuty);
}

void RemoteControl::right()
{
  // Pivot right: left wheel forward, right wheel backward
  setLeftMotor(true, m_config.motorSpeedDuty);
  setRightMotor(false, m_config.motorSpeedDuty);
}

void RemoteControl::setSpeed(uint8_t duty)
{
  m_config.motorSpeedDuty = duty;
}

bool RemoteControl::isBluetoothConnected()
{
  return m_btSerial.hasClient();
}

void RemoteControl::setLeftMotor(const bool forward, const uint8_t duty)
{
  digitalWrite(m_config.pinLeftIn1, forward ? HIGH : LOW);
  digitalWrite(m_config.pinLeftIn2, forward ? LOW : HIGH);
  ledcWrite(m_config.pwmChannelLeft, duty);
}

void RemoteControl::setRightMotor(const bool forward, const uint8_t duty)
{
  digitalWrite(m_config.pinRightIn1, forward ? HIGH : LOW);
  digitalWrite(m_config.pinRightIn2, forward ? LOW : HIGH);
  ledcWrite(m_config.pwmChannelRight, duty);
}
