#include <Arduino.h>
#include <RobotControl.h>
#include <RemoteControl.h>
#include <LineFollower.h>

static RemoteControl g_remoteControl;
static LineFollower g_lineFollower;

static RobotMode s_prevMode = RobotMode::MANUAL;

void setup()
{
  Serial.begin(115200);

  RobotControl::begin(RobotMode::MANUAL, 160U, 200U);

  g_remoteControl.begin();
  g_remoteControl.setHoldTimeout(0);
  g_lineFollower.begin(false);

  Serial.println("=== LineFollower (3xIR 4/5/15, middle black) + Remote ===");
  RobotControl::printState(Serial);
  Serial.println("Line: 3xIR digital L=4 M=5 R=15 (HIGH=black), centered=middle black+sides white, straight 160 pivot 128.");
  Serial.println("Lost line (all white) -> reverse a little + forward a little till black found.");
  Serial.println("IR values stream every loop; all white -> motors STOP.");
  Serial.println("GamePad (Arduino Bluetooth Controller):");
  Serial.println("  D-Pad Up/U/F=forward Down/D/B=back Left=L Right=R G/I/H/J=diagonals Z=stop");
  Serial.println("  X (Cross)/T -> TOGGLE MANUAL<->AUTO  A->AUTO M->MANUAL");
  Serial.println("  C (Circle)/Q/+ = +10, E/- = -10, S (Square): STOP when moving / -10 when stopped");
  Serial.println("  Speed buttons affect BOTH modes live (shared global). AUTO burst 10ms.");
  Serial.println("  USB: X/T/A/M/C/Q/E/S/Z/P also work");
}

void loop()
{

  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == 'P' || c == 'p')
    {
      g_lineFollower.printSensors(Serial);
      RobotControl::printState(Serial);
      g_remoteControl.printBTInfo(Serial);
      continue;
    }

    bool wasGlobal = RobotControl::handleCommand(c);
    if (wasGlobal)
    {

      g_remoteControl.setSpeed(RobotControl::getSpeed());
      g_lineFollower.setBaseSpeed(RobotControl::getSpeed());

      Serial.print("-> ");
      RobotControl::printState(Serial);

      if (RobotControl::isManual())
      {
        g_lineFollower.stop();
      }
      else
      {
        g_remoteControl.stop();
      }
      continue;
    }

    if (RobotControl::isManual())
    {
      g_remoteControl.handleCommand(c);
    }
    else
    {

      if (c == 'S' || c == 's' || c == 'Z' || c == 'z')
      {
        g_remoteControl.handleCommand(c);
      }
    }
  }

  g_remoteControl.update();

  RobotMode curMode = RobotControl::getMode();
  if (s_prevMode == RobotMode::MANUAL && curMode == RobotMode::LINE_FOLLOWER)
  {
    uint8_t spd = RobotControl::getSpeed();
    g_remoteControl.stop();
    g_lineFollower.startAuto();
    Serial.print("-> AUTO line follow at speed ");
    Serial.println(spd);
    g_remoteControl.getBTStream().print("AUTO speed ");
    g_remoteControl.getBTStream().println(spd);
  }
  else if (s_prevMode == RobotMode::LINE_FOLLOWER && curMode == RobotMode::MANUAL)
  {
    g_lineFollower.cancelAuto();
    Serial.println("-> MANUAL");
  }
  s_prevMode = curMode;

  RobotControl::updateLed();

  if (RobotControl::isLineFollower())
  {
    g_lineFollower.update();
  }
  else
  {

    if (RobotControl::isLineFollower())
    {
      g_remoteControl.stop();
    }
  }

  delay(10U);
}
