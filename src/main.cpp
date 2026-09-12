#include <Arduino.h>
#include <RobotControl.h>
#include <RemoteControl.h>
#include <LineFollower.h>

// Global instances - motors share same L298N pins/channels (26/27/25/14/12/33, ch 0/1)
// RobotControl holds single source of truth: g_mode + g_speed
static RemoteControl g_remoteControl;
static LineFollower g_lineFollower;

void setup()
{
  Serial.begin(115200);

  // Init global state: MANUAL + shared speed (syncs to both subsystems)
  RobotControl::begin(RobotMode::MANUAL, 120U, 200U);

  // Init motors once via RemoteControl; LineFollower reuses same pins
  g_remoteControl.begin();      // syncs motorSpeedDuty from RobotControl::g_speed
  g_lineFollower.begin(false);  // false = don't re-init motors, syncs baseSpeed from global

  Serial.println("=== LineFollower + Remote ===");
  RobotControl::printState(Serial);
  Serial.println("Commands (USB Serial OR Bluetooth):");
  Serial.println("  X/x or T/t -> TOGGLE mode (X button on controller)");
  Serial.println("  A/a -> LINE_FOLLOWER, M/m -> MANUAL");
  Serial.println("  +/- (or U/D) -> speed +/-10, 0..9 -> preset speed");
  Serial.println("  F/B/L/R/S -> manual motion (hold-to-run, auto-stop 250ms)");
  Serial.println("  P -> print IR sensors");
}

void loop()
{
  // ---- Handle USB Serial for global mode/speed (also works via BT through RemoteControl) ----
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
    // Try global handle first
    bool wasGlobal = RobotControl::handleCommand(c);
    if (wasGlobal)
    {
      // Propagate global speed to both libs (they also auto-sync, but force now)
      g_remoteControl.setSpeed(RobotControl::getSpeed());
      g_lineFollower.setBaseSpeed(RobotControl::getSpeed());

      Serial.print("-> ");
      RobotControl::printState(Serial);

      // Safety stop on mode switch
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
    // Otherwise forward to RemoteControl motion if in MANUAL
    if (RobotControl::isManual())
    {
      g_remoteControl.handleCommand(c);
    }
  }

  // ---- Run active mode ----
  if (RobotControl::isLineFollower())
  {
    g_lineFollower.update(); // IR detect + PID correction; auto-syncs speed from global
  }
  else
  {
    // In MANUAL, RemoteControl handles BT internally (including A/M/+/- mode/speed switches)
    // which already sync g_speed and stop on mode change.
    g_remoteControl.update();

    // Detect mode switch that happened via Bluetooth inside RemoteControl::handleCommand
    // If switched to LINE_FOLLOWER via BT, stop remote immediately
    if (RobotControl::isLineFollower())
    {
      g_remoteControl.stop();
    }
  }

  delay(10U);
}
