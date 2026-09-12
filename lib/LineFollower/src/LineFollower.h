#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief IR line detection + PID path correction for ESP32 + L298N.
 *
 * Default: 2-sensor black-line follower, controller-adjustable speeds:
 *   pins {36 (left), 39 (right)} (ADC1, no conflict with motors 26/27/25/14/12/33)
 *   analog + threshold 2000, HIGH = black = line (no invert)
 *   straight 100 / pivot 80 defaults, Circle(+10)/Square(-10) adjust live (shared global).
 *   baseSpeed 80 straight, turnSpeed 100 on turns.
 *
 * 2-sensor logic (black line HIGH, values stream on Serial):
 *   L+R on line (both black) -> forward base/base (intersection)
 *   neither on line (both white = all white) -> STOP motors (line lost)
 *   left only -> slow pivot left: wheels opposite at turnSpeed
 *   right only -> slow pivot right: wheels opposite at turnSpeed
 * 5/3-sensor mode still uses PID: left = base - correction, right = base + correction.
 *
 * AUTO entry: call startAuto() on MANUAL->LINE_FOLLOWER switch. update() drives a
 * short forward burst (autoBurstMs, default 10ms) then PID.
 *
 * Usage:
 *   LineFollower lf; // 2 sensors, black line, straight 100 / pivot 80, controller-adjustable
 *   lf.begin(false); // false = motors already inited by RemoteControl
 *   onModeSwitchToAuto { lf.startAuto(); }
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

    // Behavior (follows shared controller global; Circle/Square adjust live)
    uint8_t baseSpeed;      // 0..255 forward PWM when centered (default 100)
    uint8_t turnSpeed;      // 0..255 pivot speed, 2-sensor mode, both wheels opposite (default 80)
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
    uint32_t autoBurstMs;  // forward nudge on startAuto() before PID (default 10ms)
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
  void update(); // burst (if active) else readSensors + PID + drive (call every loop)
  void startAuto();  // begin AUTO: short forward burst at global speed, then PID
  void cancelAuto(); // abort burst (e.g. toggled back to MANUAL mid-burst)
  bool isBurstActive() const { return m_burstActive; }
  void setBaseSpeed(uint8_t speed); // syncs shared controller global
  void setTurnSpeed(uint8_t speed); // pivot speed, 2-sensor mode
  void setPID(float kp, float ki, float kd);
  void setThreshold(uint16_t th) { m_config.analogThreshold = th; }
  void setAutoBurstMs(uint32_t ms) { m_config.autoBurstMs = ms; }

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

  // AUTO-entry forward burst state (owned by this lib, not main.cpp)
  bool m_burstActive = false;
  uint32_t m_burstStartMs = 0U;
};
