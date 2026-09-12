#include <Arduino.h>
#include <RobotControl.h>
#include <RemoteControl.h>
#include <LineFollower.h>

// Global instances - motors share same L298N pins/channels (26/27/25/14/12/33, ch 0/1)
// RobotControl holds single source of truth: g_mode + g_speed
static RemoteControl g_remoteControl;
static LineFollower g_lineFollower;

// Track mode transition MANUAL -> AUTO for 2s forward burst
static RobotMode s_prevMode = RobotMode::MANUAL;
static bool s_autoForwardActive = false;
static uint32_t s_autoForwardStartMs = 0U;
static constexpr uint32_t AUTO_FORWARD_DURATION_MS = 2000UL;

void setup()
{
  Serial.begin(115200);

  // Init global state: MANUAL + shared speed (syncs to both subsystems)
  RobotControl::begin(RobotMode::MANUAL, 120U, 200U);

  // Init motors once via RemoteControl; LineFollower reuses same pins
  g_remoteControl.begin();      // syncs motorSpeedDuty from RobotControl::g_speed
  g_remoteControl.setHoldTimeout(0); // press-and-hold: press F/B/L/R to move, release (S) to stop
  // use 350 for tap-tap steps, 800 for hold-to-repeat auto-stop
  g_lineFollower.begin(false);  // false = don't re-init motors, syncs baseSpeed from global

  Serial.println("=== LineFollower + Remote ===");
  RobotControl::printState(Serial);
  Serial.println("GamePad (Arduino Bluetooth Controller):");
  Serial.println("  D-Pad Up/U/F=forward Down/D/B=back Left=L Right=R G/I/H/J=diagonals Z=stop");
  Serial.println("  X (Cross)/T -> TOGGLE MANUAL<->AUTO  A->AUTO M->MANUAL");
  Serial.println("  C (Circle)/Q/+ = +10 instant persistent, E/- = -10 instant anytime");
  Serial.println("  S (Square): STOP keep-speed when moving, -10 when stopped (release sends S)");
  Serial.println("  To slow while moving use E (S would stop). Trailing 0 ignored.");
  Serial.println("  USB: X/T/A/M/C/Q/E/S/Z/P also work");
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

  // ---- Always poll BT (critical: also in AUTO so 2nd X press toggles back) ----
  // Motion drive inside handleCommand is ignored in AUTO, only X/A/M/Q/E/S work.
  g_remoteControl.update();

  // ---- Detect MANUAL -> AUTO transition (via Serial X/T/A or BT X/T/A) ----
  RobotMode curMode = RobotControl::getMode();
  if (s_prevMode == RobotMode::MANUAL && curMode == RobotMode::LINE_FOLLOWER)
  {
    // Start 2s forward burst at global speed before line PID takes over
    s_autoForwardActive = true;
    s_autoForwardStartMs = millis();
    uint8_t spd = RobotControl::getSpeed();
    g_remoteControl.stop(); // ensure clean start
    g_lineFollower.forward(spd);
    Serial.print("-> AUTO forward burst 2s at speed ");
    Serial.println(spd);
    g_remoteControl.getBTStream().print("FORWARD 2s speed ");
    g_remoteControl.getBTStream().println(spd);
  }
  else if (s_prevMode == RobotMode::LINE_FOLLOWER && curMode == RobotMode::MANUAL)
  {
    // Cancel burst if toggled back to MANUAL mid-burst
    if (s_autoForwardActive)
    {
      s_autoForwardActive = false;
      g_lineFollower.stop();
      Serial.println("-> MANUAL (cancel forward burst)");
    }
  }
  s_prevMode = curMode;

  // ---- Onboard LED: MANUAL=ON, AUTO=1s blink ----
  RobotControl::updateLed();

  // ---- Run active mode ----
  if (RobotControl::isLineFollower())
  {
    if (s_autoForwardActive)
    {
      uint32_t elapsed = millis() - s_autoForwardStartMs;
      if (elapsed < AUTO_FORWARD_DURATION_MS)
      {
        // Keep driving forward (non-blocking) - also allow early exit if line seen? keep forward as requested
        // Re-assert forward in case something stopped it
        g_lineFollower.forward(RobotControl::getSpeed());
      }
      else
      {
        s_autoForwardActive = false;
        g_lineFollower.stop();
        Serial.println("-> forward burst done, line PID starts");
      }
      // During burst, skip line PID update (pure forward)
    }
    else
    {
      g_lineFollower.update(); // IR detect + PID correction; auto-syncs speed from global
    }
  }
  else
  {
    // MANUAL: BT already polled above (motion latch + X/T/A/M/Q/E handled there).
    // Hold-timeout auto-stop also handled inside update().
    // If BT X caused switch to AUTO just now, ensure remote stopped (transition handles burst next loop)
    if (RobotControl::isLineFollower())
    {
      g_remoteControl.stop();
    }
  }

  delay(10U);
}
