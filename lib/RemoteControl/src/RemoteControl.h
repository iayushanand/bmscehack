#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <stdint.h>

class RemoteControl
{
public:
  struct Config
  {
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
    uint8_t motorSpeedDuty;
    uint32_t holdTimeoutMs;
    uint8_t turnScalePct;
    const char *btDeviceName;

    Config()
        : pinLeftIn1(26U),
          pinLeftIn2(27U),
          pinLeftEn(25U),
          pinRightIn1(14U),
          pinRightIn2(12U),
          pinRightEn(33U),
          pwmChannelLeft(0U),
          pwmChannelRight(1U),
          pwmFreqHz(1000UL),
          pwmResolutionBits(8U),
          motorSpeedDuty(160U),
          holdTimeoutMs(800UL),
          turnScalePct(60U),
          btDeviceName("BURNT_TOASTER")
    {
    }
  };

  explicit RemoteControl(const Config &config);
  RemoteControl();

  void begin();

  void begin(const char *btDeviceName, uint8_t motorSpeedDuty = 160U);

  void update();

  void handleCommand(char command);

  void stop();
  void forward();
  void backward();
  void left();
  void right();
  void forwardLeft();
  void forwardRight();
  void backwardLeft();
  void backwardRight();

  void setSpeed(uint8_t duty);
  uint8_t getSpeed() const { return m_config.motorSpeedDuty; }
  void setHoldTimeout(uint32_t ms) { m_config.holdTimeoutMs = ms; }
  uint32_t getHoldTimeout() const { return m_config.holdTimeoutMs; }
  void setTurnScale(uint8_t pct) { m_config.turnScalePct = (pct > 100 ? 100 : pct); }
  bool isBluetoothConnected();

  bool btAvailable();
  int btRead();
  Stream &getBTStream();
  void printBTInfo(Stream &out = Serial);

private:
  void setLeftMotor(bool forward, uint8_t duty);
  void setRightMotor(bool forward, uint8_t duty);
  void reapplyLastMotion();

  Config m_config;
  BluetoothSerial m_btSerial;
  volatile uint32_t m_lastMotionCommandMs = 0UL;
  bool m_initialized = false;
  char m_lastMotionCmd = 'S';
  uint32_t m_lastSpeedChangeMs = 0UL;
};
