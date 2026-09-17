#pragma once

#include <Arduino.h>
#include <stdint.h>

class LineFollower
{
public:
  static constexpr uint8_t MAX_SENSORS = 6U;

  struct Config
  {

    uint8_t sensorPins[MAX_SENSORS];
    uint8_t numSensors;

    uint8_t pinLeftIn1;
    uint8_t pinLeftIn2;
    uint8_t pinLeftEn;
    uint8_t pinRightIn1;
    uint8_t pinRightIn2;
    uint8_t pinRightEn;
    uint8_t pwmChannelLeft;
    uint8_t pwmChannelRight;
    uint32_t pwmFreqHz;
    uint8_t pwmResolutionBits;

    uint8_t baseSpeed;
    uint8_t turnSpeed;
    uint8_t maxSpeed;
    uint8_t minSpeed;
    bool invertSensorLogic;
    bool analogSensors;
    uint16_t analogThreshold;
    bool lineIsBlack;

    float kp;
    float ki;
    float kd;
    uint16_t sampleTimeMs;
    uint32_t autoBurstMs;
    bool debug;

    Config();
  };

  explicit LineFollower(const Config &config);
  LineFollower();

  void begin(bool initMotors = true);
  void end();

  void calibrate(uint16_t samples = 200U);

  void readSensors();
  bool isLineDetected() const;
  bool isLineLost() const;
  int16_t getPosition();
  int16_t getLastPosition() const { return m_lastPosition; }
  uint16_t getSensorAnalog(uint8_t idx) const;
  bool getSensorDigital(uint8_t idx) const;
  void printSensors();
  void printSensors(Stream &out) const;

  void update();
  void startAuto();
  void cancelAuto();
  bool isBurstActive() const { return m_burstActive; }
  void setBaseSpeed(uint8_t speed);
  void setTurnSpeed(uint8_t speed);
  void setPID(float kp, float ki, float kd);
  void setThreshold(uint16_t th) { m_config.analogThreshold = th; }
  void setAutoBurstMs(uint32_t ms) { m_config.autoBurstMs = ms; }

  void stop();
  void drive(int16_t leftSpeed, int16_t rightSpeed);
  void forward(uint8_t speed);
  void backward(uint8_t speed);

private:
  void setLeftMotor(bool forward, uint8_t duty);
  void setRightMotor(bool forward, uint8_t duty);
  int16_t computePosition();
  int16_t pidCompute(int16_t error);

  Config m_config;
  uint16_t m_sensorValues[MAX_SENSORS];
  bool m_digitalValues[MAX_SENSORS];
  int16_t m_lastPosition = 0;
  bool m_lineLost = true;

  float m_integral = 0.0f;
  float m_lastError = 0.0f;
  uint32_t m_lastPidMs = 0U;
  bool m_initialized = false;

  bool m_burstActive = false;
  uint32_t m_burstStartMs = 0U;

  void recoverLine();
  bool m_recoveryActive = false;
  uint8_t m_recoveryPhase = 0;
  uint32_t m_recoveryStartMs = 0U;
  int8_t m_recoveryDir = 0;
};
