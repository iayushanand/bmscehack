#include "LineFollower.h"
#include "RobotControl.h"

// ---------- Config defaults ----------
LineFollower::Config::Config()
    : numSensors(5U),
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
      baseSpeed(120U),
      maxSpeed(200U),
      minSpeed(0U),
      invertSensorLogic(false),
      analogSensors(true),
      analogThreshold(2000U),
      lineIsBlack(true),
      kp(0.08f),
      ki(0.0f),
      kd(0.15f),
      sampleTimeMs(10U),
      debug(false)
{
  // Default 5 sensors left->right on ADC1 pins (avoid strapping pins)
  sensorPins[0] = 36U; // VP
  sensorPins[1] = 39U; // VN
  sensorPins[2] = 34U;
  sensorPins[3] = 35U;
  sensorPins[4] = 32U;
  sensorPins[5] = 33U; // unused for 5 sensors (33 is motor EN, keep spare)
}

// ---------- Ctor ----------
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

// ---------- Init ----------
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

  // Sync with global speed if RobotControl already inited
  m_config.baseSpeed = RobotControl::getSpeed();
  if (m_config.maxSpeed > RobotControl::getMaxSpeed())
  {
    m_config.maxSpeed = RobotControl::getMaxSpeed();
  }

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

  // Simple threshold: midpoint between min and max averaged across sensors
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

// ---------- Sensors ----------
void LineFollower::readSensors()
{
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    if (m_config.analogSensors)
    {
      m_sensorValues[i] = analogRead(m_config.sensorPins[i]);
      bool onLine = (m_sensorValues[i] > m_config.analogThreshold);
      if (m_config.invertSensorLogic) onLine = !onLine;
      // lineIsBlack handling: analog high = white reflective, low = black line for many IR modules
      // If module is digital inverted, use invertSensorLogic. Keep generic.
      m_digitalValues[i] = onLine;
    }
    else
    {
      int v = digitalRead(m_config.sensorPins[i]);
      m_digitalValues[i] = m_config.invertSensorLogic ? (v == LOW) : (v == HIGH);
      m_sensorValues[i] = m_digitalValues[i] ? 4095U : 0U;
    }
  }
  // Update last known position if line visible, else keep last
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
    out.print(m_config.analogSensors ? m_sensorValues[i] : (m_digitalValues[i] ? 1 : 0));
    out.print(i < m_config.numSensors - 1 ? " | " : "");
  }
  out.print("  pos=");
  out.print(m_lastPosition);
  out.print(m_lineLost ? " LOST" : " OK");
  out.println();
}

// Weighted position: -2000..+2000 for 5 sensors, scaled for 3 sensors similarly
int16_t LineFollower::computePosition()
{
  uint8_t active = 0U;
  int32_t weightedSum = 0;
  int32_t sum = 0;

  // Weight map for up to 6 sensors centered at 0
  // For 5: -2,-1,0,1,2 -> scaled *1000
  // For 3: -1,0,1 -> scaled *1000
  // Generic: index - (n-1)/2
  for (uint8_t i = 0; i < m_config.numSensors; i++)
  {
    if (m_digitalValues[i])
    {
      active++;
      int16_t weight = (int16_t)i - (int16_t)(m_config.numSensors - 1) / 2;
      // For even counts, adjust half-step: 4 sensors -> -1.5,-0.5,0.5,1.5
      // Multiply by 1000 for resolution
      int16_t w1000;
      if (m_config.numSensors % 2 == 1)
      {
        w1000 = weight * 1000;
      }
      else
      {
        // even: weight is offset by 0.5
        w1000 = weight * 1000 + (weight >= 0 ? 500 : -500);
        // correction for 0 missing
        if (m_config.numSensors == 4 && i >= 2) w1000 -= 1000;
      }
      weightedSum += (int32_t)w1000;
      sum += 1000;
    }
  }

  if (active == 0U)
  {
    m_lineLost = true;
    // Return last position to allow recovery (spin toward last seen line)
    return m_lastPosition;
  }

  m_lineLost = false;
  // Average weight, already in -2000..2000 range for 5 sensors (weight*1000)
  int16_t pos = (int16_t)(weightedSum * 1000 / sum);
  // Normalize: for 5 sensors max 2000 already, for 3 sensors max 1000 -> scale to 2000
  if (m_config.numSensors == 3)
  {
    pos *= 2;
  }
  return pos;
}

// ---------- Control ----------
int16_t LineFollower::pidCompute(int16_t error)
{
  uint32_t now = millis();
  uint32_t dtMs = now - m_lastPidMs;
  if (dtMs < m_config.sampleTimeMs) dtMs = m_config.sampleTimeMs;
  float dt = dtMs / 1000.0f;

  m_integral += error * dt;
  // anti-windup clamp
  const float integralLimit = 1000.0f;
  if (m_integral > integralLimit) m_integral = integralLimit;
  if (m_integral < -integralLimit) m_integral = -integralLimit;

  float derivative = (error - m_lastError) / dt;

  float out = m_config.kp * error + m_config.ki * m_integral + m_config.kd * derivative;

  m_lastError = error;
  m_lastPidMs = now;

  // Clamp to PWM range
  if (out > 255) out = 255;
  if (out < -255) out = -255;
  return (int16_t)out;
}

void LineFollower::update()
{
  if (!m_initialized) return;

  // Sync global speed (single source of truth)
  if (m_config.baseSpeed != RobotControl::getSpeed())
  {
    m_config.baseSpeed = RobotControl::getSpeed();
  }

  readSensors();

  if (m_config.debug)
  {
    printSensors(Serial);
  }

  int16_t pos = getPosition();

  if (isLineLost())
  {
    // Line lost: stop or spin toward last position to reacquire
    // Simple strategy: pivot gently toward last known side
    if (m_lastPosition < 0)
    {
      // line was to left -> turn left slowly
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

  int16_t error = pos; // target 0
  int16_t correction = pidCompute(error);

  int16_t left = (int16_t)m_config.baseSpeed - correction;
  int16_t right = (int16_t)m_config.baseSpeed + correction;

  // Clamp
  if (left > m_config.maxSpeed) left = m_config.maxSpeed;
  if (right > m_config.maxSpeed) right = m_config.maxSpeed;
  if (left < - (int16_t)m_config.maxSpeed) left = - (int16_t)m_config.maxSpeed;
  if (right < - (int16_t)m_config.maxSpeed) right = - (int16_t)m_config.maxSpeed;

  // Apply minSpeed deadzone: if small PWM won't move, either 0 or at least minSpeed
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

void LineFollower::setPID(float kp, float ki, float kd)
{
  m_config.kp = kp;
  m_config.ki = ki;
  m_config.kd = kd;
}

// ---------- Motors ----------
void LineFollower::stop()
{
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
  // left
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
