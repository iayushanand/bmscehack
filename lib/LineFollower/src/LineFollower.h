#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief IR line detection + PID path correction for ESP32 + L298N.
 *
 * Supports 3 or 5 channel IR array (analog or digital). Default 5 sensors:
 *   pins {36, 39, 34, 35, 32} left -> right (ADC1, no conflict with RemoteControl motors 26/27/25/14/12/33)
 *
 * Line position is weighted average: [-2000 .. +2000] for 5 sensors (0 = centered).
 * PID correction drives differential motors: left = base - correction, right = base + correction.
 *
 * Usage:
 *   LineFollower lf;
 *   lf.begin();
 *   lf.calibrate(); // optional auto-threshold
 *   loop { lf.update(); }
 *
 * Coexists with RemoteControl lib (shares same L298N pins/channels 0,1).
 * Call only ONE of RemoteControl::begin() or LineFollower::begin() to init motors,
 * or call LineFollower::begin(false) to skip motor init if RemoteControl already did.
 */
class LineFollower
{
public:
  static constexpr uint8_t MAX_SENSORS = 6U;

  struct Config
  {
    // IR sensor pins (left -> right)
    uint8_t sensorPins[MAX_SENSORS];
    uint8_t numSensors;

    // Motor pins (must match RemoteControl defaults if both libs used)
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

    // Behavior
    uint8_t baseSpeed;      // 0..255 forward PWM when centered
    uint8_t maxSpeed;       // clamp
    uint8_t minSpeed;       // minimum to overcome friction (applied as 0 or >=min)
    bool invertSensorLogic; // false: HIGH/dark = line (digital), or analog > threshold = line
    bool analogSensors;     // true: use analogRead + threshold, false: digitalRead
    uint16_t analogThreshold; // 0..4095 threshold for analog mode
    bool lineIsBlack;       // affects position calc if needed (kept for future)
    // PID
    float kp;
    float ki;
    float kd;
    uint16_t sampleTimeMs; // PID update interval
    bool debug;

    Config();
  };

  explicit LineFollower(const Config &config);
  LineFollower();

  /** Init IR pins + L298N motors. Set initMotors=false if RemoteControl already inited motors. */
  void begin(bool initMotors = true);
  void end();

  /** Auto-calibrate threshold from current surface (call on white then black, or just samples). */
  void calibrate(uint16_t samples = 200U);

  // ---- Sensor API ----
  void readSensors(); // updates m_sensorValues / m_digitalValues
  bool isLineDetected() const;
  bool isLineLost() const;
  int16_t getPosition(); // -2000..+2000 (0 centered), 0 if line lost
  int16_t getLastPosition() const { return m_lastPosition; }
  uint16_t getSensorAnalog(uint8_t idx) const;
  bool getSensorDigital(uint8_t idx) const;
  void printSensors(); // prints to Serial
  void printSensors(Stream &out) const;

  // ---- Control API ----
  void update(); // readSensors + PID + drive (call every loop)
  void setBaseSpeed(uint8_t speed);
  void setPID(float kp, float ki, float kd);
  void setThreshold(uint16_t th) { m_config.analogThreshold = th; }

  // Motor primitives (same as RemoteControl)
  void stop();
  void drive(int16_t leftSpeed, int16_t rightSpeed); // -255..255 (negative = backward)
  void forward(uint8_t speed);
  void backward(uint8_t speed);

private:
  void setLeftMotor(bool forward, uint8_t duty);
  void setRightMotor(bool forward, uint8_t duty);
  int16_t computePosition(); // weighted
  int16_t pidCompute(int16_t error);

  Config m_config;
  uint16_t m_sensorValues[MAX_SENSORS];
  bool m_digitalValues[MAX_SENSORS];
  int16_t m_lastPosition = 0;
  bool m_lineLost = true;

  // PID state
  float m_integral = 0.0f;
  float m_lastError = 0.0f;
  uint32_t m_lastPidMs = 0U;
  bool m_initialized = false;
};
