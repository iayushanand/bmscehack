#include "RemoteControl.h"
#include "RobotControl.h"

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

  m_config.motorSpeedDuty = RobotControl::getSpeed();

  stop();

  (void)m_btSerial.begin(m_config.btDeviceName);
  m_btSerial.println("Bluetooth GamePad ready (Arduino Bluetooth Controller)");
  m_btSerial.println("F/U=fwd B/D=back L/R=turn G/I/H/J=diag Z=stop");
  m_btSerial.println("X=toggle MANUAL<->AUTO, C/Q/+=faster S/E/-=slower (persistent)");

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

    Serial.print("[BT] RX: '");
    Serial.print(cmd);
    Serial.print("' (0x");
    Serial.print((uint8_t)cmd, HEX);
    Serial.print(") mode=");
    Serial.print(RobotControl::modeToString());
    Serial.print(" speed=");
    Serial.println(RobotControl::getSpeed());

    if (cmd == 'X' || cmd == 'x' || cmd == 'Y' || cmd == 'y')
    {

      if (m_btSerial.available() > 0 && m_btSerial.peek() == ':')
      {
        String line;
        line += cmd;

        uint32_t t0 = millis();
        while (m_btSerial.available() > 0 && millis() - t0 < 20)
        {
          char c2 = (char)m_btSerial.read();
          line += c2;
          if (c2 == '\n') break;
        }

        int xVal = 0, yVal = 0;
        int xi = line.indexOf('X');
        if (xi < 0) xi = line.indexOf('x');
        int yi = line.indexOf('Y');
        if (yi < 0) yi = line.indexOf('y');
        if (xi >= 0)
        {
          xVal = line.substring(xi + 2).toInt();

        }
        if (yi >= 0)
        {
          yVal = line.substring(yi + 2).toInt();
        }

        Serial.print("[BT] Joystick X:");
        Serial.print(xVal);
        Serial.print(" Y:");
        Serial.print(yVal);
        Serial.print(" raw:'");
        Serial.print(line);
        Serial.println("'");

        if (RobotControl::isLineFollower())
        {
          Serial.println("[BT] Joystick IGNORED in AUTO (press X for MANUAL)");
          continue;
        }
        if (abs(xVal) < 5 && abs(yVal) < 5)
        {
          Serial.println("[BT] Joystick center -> STOP");
          stop();
          m_lastMotionCommandMs = 0UL;
          m_lastMotionCmd = 'S';
        }
        else
        {
          m_config.motorSpeedDuty = RobotControl::getSpeed();
          int16_t base = (int16_t)(yVal * m_config.motorSpeedDuty / 100);
          int16_t turn = (int16_t)(xVal * m_config.motorSpeedDuty / 100);
          int16_t left = base - turn;
          int16_t right = base + turn;

          if (left > 255) left = 255;
          if (left < -255) left = -255;
          if (right > 255) right = 255;
          if (right < -255) right = -255;
          Serial.print("[BT] Drive L:");
          Serial.print(left);
          Serial.print(" R:");
          Serial.println(right);

          if (left >= 0) setLeftMotor(true, (uint8_t)left);
          else setLeftMotor(false, (uint8_t)-left);
          if (right >= 0) setRightMotor(true, (uint8_t)right);
          else setRightMotor(false, (uint8_t)-right);
          m_lastMotionCommandMs = millis();
        }
        continue;
      }
    }
    handleCommand(cmd);
  }

  if (m_lastMotionCommandMs > 0UL && m_config.holdTimeoutMs > 0UL)
  {
    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - m_lastMotionCommandMs;
    if (elapsedMs > m_config.holdTimeoutMs)
    {
      stop();
      m_lastMotionCommandMs = 0UL;
      m_lastMotionCmd = 'S';
    }
  }
}

void RemoteControl::handleCommand(const char command)
{

  RobotMode before = RobotControl::getMode();
  if (RobotControl::handleCommand(command))
  {

    m_config.motorSpeedDuty = RobotControl::getSpeed();
    RobotMode after = RobotControl::getMode();
    if (before != after)
    {

      stop();
      m_lastMotionCommandMs = 0UL;
      m_lastMotionCmd = 'S';
      m_btSerial.print("MODE -> ");
      m_btSerial.println(RobotControl::modeToString(after));
      Serial.print("[Remote] X toggle -> ");
      Serial.println(RobotControl::modeToString(after));
    }
    else
    {

      m_lastSpeedChangeMs = millis();
      m_btSerial.print("SPEED -> ");
      m_btSerial.println(RobotControl::getSpeed());
      if (command == 'C' || command == 'c')
      {
        Serial.print("[Remote] Circle +10 -> ");
        Serial.println(RobotControl::getSpeed());
      }
      else
      {
        Serial.print("[Remote] speed -> ");
        Serial.println(RobotControl::getSpeed());
      }

      if (RobotControl::isManual() && m_lastMotionCommandMs != 0UL && m_lastMotionCmd != 'S')
      {
        reapplyLastMotion();
      }
    }
    return;
  }

  if (RobotControl::isLineFollower() && command != 'S' && command != 's' && command != 'Z' && command != 'z')
  {
    Serial.print("[BT] handle '");
    Serial.print(command);
    Serial.println("' -> IGNORED in AUTO (press X for MANUAL)");
    return;
  }

  Serial.print("[BT] handle '");
  Serial.print(command);
  Serial.print("' -> ");
  switch (command)
  {

    case 'F':
    case 'f':
    case 'U':
    case 'u':
      Serial.println("FORWARD");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      forward();
      m_lastMotionCmd = 'F';
      m_lastMotionCommandMs = millis();
      break;

    case 'B':
    case 'b':
    case 'D':
    case 'd':
      Serial.println("BACKWARD");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      backward();
      m_lastMotionCmd = 'B';
      m_lastMotionCommandMs = millis();
      break;

    case 'L':
    case 'l':
      Serial.println("LEFT spin");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      left();
      m_lastMotionCmd = 'L';
      m_lastMotionCommandMs = millis();
      break;

    case 'R':
    case 'r':
      Serial.println("RIGHT spin");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      right();
      m_lastMotionCmd = 'R';
      m_lastMotionCommandMs = millis();
      break;

    case 'G':
    case 'g':
      Serial.println("FORWARD-LEFT");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      forwardLeft();
      m_lastMotionCmd = 'G';
      m_lastMotionCommandMs = millis();
      break;
    case 'I':
    case 'i':
      Serial.println("FORWARD-RIGHT");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      forwardRight();
      m_lastMotionCmd = 'I';
      m_lastMotionCommandMs = millis();
      break;
    case 'H':
    case 'h':
      Serial.println("BACKWARD-LEFT");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      backwardLeft();
      m_lastMotionCmd = 'H';
      m_lastMotionCommandMs = millis();
      break;
    case 'J':
    case 'j':
      Serial.println("BACKWARD-RIGHT");
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      backwardRight();
      m_lastMotionCmd = 'J';
      m_lastMotionCommandMs = millis();
      break;

    case 'Z':
    case 'z':
      Serial.println("STOP (Z)");
      stop();
      m_lastMotionCmd = 'S';
      m_lastMotionCommandMs = 0UL;
      break;

    case '0':
    case '\0':

      if (!RobotControl::isManual())
      {
        Serial.println("RELEASE ignored in AUTO");
        break;
      }
      if (m_lastMotionCommandMs != 0UL && (millis() - m_lastSpeedChangeMs > 250UL))
      {
        Serial.println("RELEASE stop (speed kept)");
        Serial.print("speed kept at ");
        Serial.println(RobotControl::getSpeed());
        stop();
        m_lastMotionCmd = 'S';
        m_lastMotionCommandMs = 0UL;
      }
      else
      {
        Serial.println("RELEASE suffix ignored (after speed btn)");
      }
      break;

    case 'S':
    case 's':

      RobotControl::changeSpeed(-10);
      m_config.motorSpeedDuty = RobotControl::getSpeed();
      m_lastSpeedChangeMs = millis();
      Serial.print("SQUARE -10 -> ");
      Serial.println(RobotControl::getSpeed());
      m_btSerial.print("SPEED -> ");
      m_btSerial.println(RobotControl::getSpeed());
      if (RobotControl::isManual() && m_lastMotionCommandMs != 0UL && m_lastMotionCmd != 'S')
      {
        reapplyLastMotion();
      }
      break;

    default:
      Serial.println("IGNORED");

      break;
  }
}

void RemoteControl::reapplyLastMotion()
{

  m_config.motorSpeedDuty = RobotControl::getSpeed();
  switch (m_lastMotionCmd)
  {
    case 'F': forward(); break;
    case 'B': backward(); break;
    case 'L': left(); break;
    case 'R': right(); break;
    case 'G': forwardLeft(); break;
    case 'I': forwardRight(); break;
    case 'H': backwardLeft(); break;
    case 'J': backwardRight(); break;
    default: return;
  }
  m_lastMotionCommandMs = millis();
  Serial.print("[Remote] reapply ");
  Serial.print(m_lastMotionCmd);
  Serial.print(" at ");
  Serial.println(m_config.motorSpeedDuty);
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

  uint8_t t = (uint8_t)(m_config.motorSpeedDuty * m_config.turnScalePct / 100U);
  setLeftMotor(false, t);
  setRightMotor(true, t);
}

void RemoteControl::right()
{

  uint8_t t = (uint8_t)(m_config.motorSpeedDuty * m_config.turnScalePct / 100U);
  setLeftMotor(true, t);
  setRightMotor(false, t);
}

void RemoteControl::forwardLeft()
{

  uint8_t left = (uint8_t)(m_config.motorSpeedDuty * 0.4f);
  setLeftMotor(true, left);
  setRightMotor(true, m_config.motorSpeedDuty);
}

void RemoteControl::forwardRight()
{
  uint8_t right = (uint8_t)(m_config.motorSpeedDuty * 0.4f);
  setLeftMotor(true, m_config.motorSpeedDuty);
  setRightMotor(true, right);
}

void RemoteControl::backwardLeft()
{
  uint8_t left = (uint8_t)(m_config.motorSpeedDuty * 0.4f);
  setLeftMotor(false, left);
  setRightMotor(false, m_config.motorSpeedDuty);
}

void RemoteControl::backwardRight()
{
  uint8_t right = (uint8_t)(m_config.motorSpeedDuty * 0.4f);
  setLeftMotor(false, m_config.motorSpeedDuty);
  setRightMotor(false, right);
}

void RemoteControl::setSpeed(uint8_t duty)
{
  m_config.motorSpeedDuty = duty;

  RobotControl::setSpeed(duty);
}

bool RemoteControl::btAvailable()
{
  return m_btSerial.available() > 0;
}

int RemoteControl::btRead()
{
  return m_btSerial.read();
}

Stream &RemoteControl::getBTStream()
{
  return m_btSerial;
}

void RemoteControl::printBTInfo(Stream &out)
{
  out.print("BT ");
  out.print(m_config.btDeviceName);
  out.print(m_btSerial.hasClient() ? " connected" : " idle");
  out.print(" speed=");
  out.println(m_config.motorSpeedDuty);
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
