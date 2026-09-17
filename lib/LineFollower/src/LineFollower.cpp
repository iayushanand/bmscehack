#include "LineFollower.h"
#include "RobotControl.h"

LineFollower::Config::Config()
    : numSensors(3U),
      pinLeftIn1(26U),
      pinLeftIn2(27U),
      pinLeftEn(25U),
      pinRightIn1(14U),
      pinRightIn2(12U),
      pinRightEn(33U),
      pwmChannelLeft(0U),
      pwmChannelRight(1U),
      pwmFreqHz(1000UL),
      pwmResolutionBits(8U),
      baseSpeed(160U),
      turnSpeed(128U),
      maxSpeed(200U),
      minSpeed(0U),
      invertSensorLogic(false),
      analogSensors(false),
      analogThreshold(2000U),
      lineIsBlack(true),
      kp(0.12f),
      ki(0.0f),
      kd(0.18f),
      sampleTimeMs(10U),
      autoBurstMs(10UL),
      debug(true)
{

  sensorPins[0] = 4U;
  sensorPins[1] = 5U;
  sensorPins[2] = 15U;
  sensorPins[3] = 34U;
  sensorPins[4] = 35U;
  sensorPins[5] = 32U;
}

LineFollower::LineFollower(const Config &config)
    : m_config(config)
{
  for (uint8_t i = 0; i < MAX_SENSORS; i++)
  {
    m_sensorValues[i] = 0U;
    m_digitalValues[i] = false;
  }
}

LineFollower::LineFollower()
    : m_config(Config())
{
  for (uint8_t i = 0; i < MAX_SENSORS; i++)
  {
    m_sensorValues[i] = 0U;
    m_digitalValues[i] = false;
  }
}

void LineFollower::begin(bool initMotors)
{
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    pinMode(m_config.sensorPins[i], INPUT);
  }

  if (initMotors)
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
  }

  m_config.baseSpeed = RobotControl::getSpeed();
  m_lastPidMs = millis();
  m_initialized = true;

  if (m_config.debug)
  {
    Serial.println("[LineFollower] begin");
    Serial.print("  sensors=");
    Serial.print(m_config.numSensors);
    Serial.print(" analog=");
    Serial.print(m_config.analogSensors ? "yes" : "no");
    Serial.print(" th=");
    Serial.println(m_config.analogThreshold);
  }
}

void LineFollower::end()
{
  stop();
  m_initialized = false;
}

void LineFollower::calibrate(uint16_t samples)
{
  if (m_config.numSensors == 0U) return;
  uint32_t sum[MAX_SENSORS] = {0};
  uint16_t minV[MAX_SENSORS];
  uint16_t maxV[MAX_SENSORS];
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    minV[i] = 4095U;
    maxV[i] = 0U;
  }

  for (uint16_t s = 0; s < samples; s++)
  {
    for (uint8_t i = 0; i < m_config.numSensors; i++)
    {
      uint16_t v = m_config.analogSensors ? analogRead(m_config.sensorPins[i]) : (digitalRead(m_config.sensorPins[i]) ? 4095U : 0U);
      sum[i] += v;
      if (v < minV[i]) minV[i] = v;
      if (v > maxV[i]) maxV[i] = v;
    }
    delay(5U);
  }

  uint32_t avgMin = 0, avgMax = 0;
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    avgMin += minV[i];
    avgMax += maxV[i];
  }
  avgMin /= m_config.numSensors;
  avgMax /= m_config.numSensors;
  m_config.analogThreshold = (uint16_t)((avgMin + avgMax) / 2U);

  if (m_config.debug)
  {
    Serial.print("[LineFollower] calibrate threshold=");
    Serial.println(m_config.analogThreshold);
  }
}

void LineFollower::readSensors()
{
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    if (m_config.analogSensors)
    {
      m_sensorValues[i] = analogRead(m_config.sensorPins[i]);
      bool onLine = (m_sensorValues[i] > m_config.analogThreshold);
      if (m_config.invertSensorLogic) onLine = !onLine;

      m_digitalValues[i] = onLine;
    }
    else
    {
      int v = digitalRead(m_config.sensorPins[i]);
      m_digitalValues[i] = m_config.invertSensorLogic ? (v == LOW) : (v == HIGH);
      m_sensorValues[i] = m_digitalValues[i] ? 4095U : 0U;
    }
  }

  int16_t pos = computePosition();
  if (!m_lineLost)
  {
    m_lastPosition = pos;
  }
}

bool LineFollower::isLineDetected() const
{
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    if (m_digitalValues[i]) return true;
  }
  return false;
}

bool LineFollower::isLineLost() const
{
  return m_lineLost;
}

int16_t LineFollower::getPosition()
{
  return m_lineLost ? m_lastPosition : computePosition();
}

void LineFollower::printSensors()
{
  printSensors(Serial);
}

uint16_t LineFollower::getSensorAnalog(uint8_t idx) const
{
  if (idx >= m_config.numSensors) return 0U;
  return m_sensorValues[idx];
}

bool LineFollower::getSensorDigital(uint8_t idx) const
{
  if (idx >= m_config.numSensors) return false;
  return m_digitalValues[idx];
}

void LineFollower::printSensors(Stream &out) const
{
  out.print("Sensors [L->R]: ");
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    if (m_config.analogSensors)
    {

      int d = digitalRead(m_config.sensorPins[i]);
      out.print(m_sensorValues[i]);
      out.print("(D");
      out.print(d);
      out.print(")");
    }
    else
    {
      out.print(m_digitalValues[i] ? 1 : 0);
    }
    out.print(i < m_config.numSensors - 1 ? " | " : "");
  }
  out.print("  pos=");
  out.print(m_lastPosition);
  out.print(m_lineLost ? " LOST" : " OK");
  out.println();
}

int16_t LineFollower::computePosition()
{

  if (m_config.numSensors == 2U)
  {
    bool L = m_digitalValues[0];
    bool R = m_digitalValues[1];
    if (L && !R) { m_lineLost = false; return -1000; }
    if (!L && R) { m_lineLost = false; return 1000; }
    if (L && R) { m_lineLost = false; return 0; }
    m_lineLost = true;
    return m_lastPosition;
  }

  uint8_t active = 0U;
  int32_t weightedSum = 0;
  int32_t sum = 0;

  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    if (m_digitalValues[i])
    {
      active++;
      int16_t weight = (int16_t)i - (int16_t)(m_config.numSensors - 1) / 2;

      int16_t w1000;
      if (m_config.numSensors % 2 == 1)
      {
        w1000 = weight * 1000;
      }
      else
      {

        w1000 = weight * 1000 + (weight >= 0 ? 500 : -500);

        if (m_config.numSensors == 4 && i >= 2) w1000 -= 1000;
      }
      weightedSum += (int32_t)w1000;
      sum += 1000;
    }
  }

  if (active == 0U)
  {
    m_lineLost = true;

    return m_lastPosition;
  }

  m_lineLost = false;

  int16_t pos = (int16_t)(weightedSum * 1000 / sum);

  if (m_config.numSensors == 3)
  {
    pos *= 2;
  }
  return pos;
}

int16_t LineFollower::pidCompute(int16_t error)
{
  uint32_t now = millis();
  uint32_t dtMs = now - m_lastPidMs;
  if (dtMs < m_config.sampleTimeMs) dtMs = m_config.sampleTimeMs;
  float dt = dtMs / 1000.0f;

  m_integral += error * dt;

  const float integralLimit = 1000.0f;
  if (m_integral > integralLimit) m_integral = integralLimit;
  if (m_integral < -integralLimit) m_integral = -integralLimit;

  float derivative = (error - m_lastError) / dt;

  float out = m_config.kp * error + m_config.ki * m_integral + m_config.kd * derivative;

  m_lastError = error;
  m_lastPidMs = now;

  if (out > 255) out = 255;
  if (out < -255) out = -255;
  return (int16_t)out;
}

void LineFollower::startAuto()
{
  if (!m_initialized) return;
  m_burstActive = true;
  m_burstStartMs = millis();
  m_config.baseSpeed = RobotControl::getSpeed();
  forward(m_config.baseSpeed);
  if (m_config.debug)
  {
    Serial.print("[LineFollower] burst ");
    Serial.print(m_config.autoBurstMs);
    Serial.print("ms at ");
    Serial.println(m_config.baseSpeed);
  }
}

void LineFollower::cancelAuto()
{
  if (m_burstActive)
  {
    m_burstActive = false;
    stop();
  }
}

void LineFollower::update()
{
  if (!m_initialized) return;

  if (m_config.baseSpeed != RobotControl::getSpeed())
  {
    uint8_t g = RobotControl::getSpeed();

    m_config.turnSpeed = (uint8_t)((uint16_t)g * 80U / 100U);
    m_config.baseSpeed = g;
  }

  if (m_burstActive)
  {
    uint32_t elapsed = millis() - m_burstStartMs;
    if (elapsed < m_config.autoBurstMs)
    {
      forward(m_config.baseSpeed);
      return;
    }
    m_burstActive = false;
    stop();
    if (m_config.debug)
    {
      Serial.println("[LineFollower] burst done, PID starts");
    }
  }

  readSensors();

  if (m_config.debug)
  {
    printSensors(Serial);
  }

  int16_t pos = getPosition();

  if (m_config.numSensors == 2U)
  {
    if (isLineLost())
    {
      recoverLine();
      return;
    }
    m_recoveryActive = false;
    if (pos == 0)
    {
      drive(m_config.baseSpeed, m_config.baseSpeed);
    }
    else if (pos < 0)
    {
      drive(-(int16_t)m_config.turnSpeed, m_config.turnSpeed);
    }
    else
    {
      drive(m_config.turnSpeed, -(int16_t)m_config.turnSpeed);
    }
    return;
  }

  if (m_config.numSensors == 3U)
  {
    if (isLineLost())
    {
      recoverLine();
      return;
    }
    m_recoveryActive = false;
    const bool L = m_digitalValues[0];
    const bool M = m_digitalValues[1];
    const bool R = m_digitalValues[2];
    const uint8_t inner = (uint8_t)(m_config.baseSpeed * 40U / 100U);
    if ((!L && M && !R) || (L && M && R) || (L && !M && R))
    {
      drive(m_config.baseSpeed, m_config.baseSpeed);
    }
    else if (L && M && !R)
    {
      drive(inner, m_config.baseSpeed);
    }
    else if (!L && M && R)
    {
      drive(m_config.baseSpeed, inner);
    }
    else if (L)
    {
      drive(-(int16_t)m_config.turnSpeed, m_config.turnSpeed);
    }
    else if (R)
    {
      drive(m_config.turnSpeed, -(int16_t)m_config.turnSpeed);
    }
    else
    {
      drive(m_config.baseSpeed, m_config.baseSpeed);
    }
    return;
  }

  if (isLineLost())
  {

    if (m_lastPosition < 0)
    {

      drive(-60, 60);
    }
    else if (m_lastPosition > 0)
    {
      drive(60, -60);
    }
    else
    {
      stop();
    }
    return;
  }

  int16_t error = pos;
  int16_t correction = pidCompute(error);

  int16_t left = (int16_t)m_config.baseSpeed - correction;
  int16_t right = (int16_t)m_config.baseSpeed + correction;

  if (left > m_config.maxSpeed) left = m_config.maxSpeed;
  if (right > m_config.maxSpeed) right = m_config.maxSpeed;
  if (left < - (int16_t)m_config.maxSpeed) left = - (int16_t)m_config.maxSpeed;
  if (right < - (int16_t)m_config.maxSpeed) right = - (int16_t)m_config.maxSpeed;

  if (m_config.minSpeed > 0)
  {
    if (left > 0 && left < m_config.minSpeed) left = m_config.minSpeed;
    if (right > 0 && right < m_config.minSpeed) right = m_config.minSpeed;
    if (left < 0 && left > - (int16_t)m_config.minSpeed) left = - (int16_t)m_config.minSpeed;
    if (right < 0 && right > - (int16_t)m_config.minSpeed) right = - (int16_t)m_config.minSpeed;
  }

  drive(left, right);
}

void LineFollower::setBaseSpeed(uint8_t speed)
{
  m_config.baseSpeed = speed;
  RobotControl::setSpeed(speed);
}

void LineFollower::setTurnSpeed(uint8_t speed)
{
  m_config.turnSpeed = speed;
}

void LineFollower::setPID(float kp, float ki, float kd)
{
  m_config.kp = kp;
  m_config.ki = ki;
  m_config.kd = kd;
}

void LineFollower::recoverLine()
{
  static constexpr uint32_t BACK_MS = 300UL;
  static constexpr uint32_t FWD_MS = 400UL;
  const uint32_t now = millis();

  if (!m_recoveryActive)
  {
    m_recoveryActive = true;
    m_recoveryPhase = 0;
    m_recoveryStartMs = now;
    m_recoveryDir = (m_lastPosition < 0) ? -1 : ((m_lastPosition > 0) ? 1 : 0);
    if (m_config.debug)
    {
      Serial.print("[LineFollower] lost -> search back/forth, last side ");
      Serial.println(m_recoveryDir);
    }
  }

  const uint32_t elapsed = now - m_recoveryStartMs;
  if (m_recoveryPhase == 0)
  {
    const int16_t s = (int16_t)(m_config.baseSpeed * 60U / 100U);
    drive(-s, -s);
    if (elapsed >= BACK_MS)
    {
      m_recoveryPhase = 1;
      m_recoveryStartMs = now;
    }
  }
  else
  {
    const uint8_t inner = (uint8_t)(m_config.baseSpeed * 40U / 100U);
    if (m_recoveryDir < 0)
    {
      drive(inner, m_config.baseSpeed);
    }
    else if (m_recoveryDir > 0)
    {
      drive(m_config.baseSpeed, inner);
    }
    else
    {
      drive(m_config.baseSpeed, m_config.baseSpeed);
    }
    if (elapsed >= FWD_MS)
    {
      m_recoveryPhase = 0;
      m_recoveryStartMs = now;
    }
  }
}

void LineFollower::stop()
{
  m_burstActive = false;
  m_recoveryActive = false;
  digitalWrite(m_config.pinLeftIn1, LOW);
  digitalWrite(m_config.pinLeftIn2, LOW);
  digitalWrite(m_config.pinRightIn1, LOW);
  digitalWrite(m_config.pinRightIn2, LOW);
  ledcWrite(m_config.pwmChannelLeft, 0U);
  ledcWrite(m_config.pwmChannelRight, 0U);
  m_integral = 0.0f;
}

void LineFollower::drive(int16_t leftSpeed, int16_t rightSpeed)
{

  if (leftSpeed >= 0)
    setLeftMotor(true, (uint8_t)leftSpeed);
  else
    setLeftMotor(false, (uint8_t)(-leftSpeed));

  if (rightSpeed >= 0)
    setRightMotor(true, (uint8_t)rightSpeed);
  else
    setRightMotor(false, (uint8_t)(-rightSpeed));
}

void LineFollower::forward(uint8_t speed)
{
  setLeftMotor(true, speed);
  setRightMotor(true, speed);
}

void LineFollower::backward(uint8_t speed)
{
  setLeftMotor(false, speed);
  setRightMotor(false, speed);
}

void LineFollower::setLeftMotor(bool forward, uint8_t duty)
{
  digitalWrite(m_config.pinLeftIn1, forward ? HIGH : LOW);
  digitalWrite(m_config.pinLeftIn2, forward ? LOW : HIGH);
  ledcWrite(m_config.pwmChannelLeft, duty);
}

void LineFollower::setRightMotor(bool forward, uint8_t duty)
{
  digitalWrite(m_config.pinRightIn1, forward ? HIGH : LOW);
  digitalWrite(m_config.pinRightIn2, forward ? LOW : HIGH);
  ledcWrite(m_config.pwmChannelRight, duty);
}
