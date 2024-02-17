package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TimerConstants;

public final class DashboardSubsystem {
  // determinew what timer is enabled
  static boolean TeleOpStartTimerEnabled = false;
  static boolean TeleOpEndTimerEnabled = false;
  static boolean AutoTimerEnabled = false;

  // Is the the ammount of times RobotPeriodic in robot.java has passed (updates every 20ms)

  // check Constants for timervalues

  public static void StartTimer(String type, double timer_length) {

    switch (type) {
      case "TeleOpStartTimer":
        TeleOpStartTimerEnabled = true;
        SmartDashboard.putNumber("TeleOpStartTimer", timer_length);
        break;
      case "TeleOpEndTimer":
        TeleOpEndTimerEnabled = true;
        SmartDashboard.putNumber("TeleOpEndTimer", timer_length);
        break;
      case "AutoTimer":
        AutoTimerEnabled = true;
        SmartDashboard.putNumber("AutoTimer", timer_length);
        break;
    }
  }

  public static void RunTimers() {
    // will run code if 1 second has passed
    if (TeleOpStartTimerEnabled) {
      if (SmartDashboard.getNumber("TeleOpStartTimer", 0) <= .02) {
        TeleOpStartTimerEnabled = false;
        // will start second timer after this ends
        StartTimer("TeleOpEndTimer", TimerConstants.TeleOpEndTimerLength);
      } else {
        SmartDashboard.putNumber(
            "TeleOpStartTimer", SmartDashboard.getNumber("TeleOpStartTimer", 0) - .02);
      }
    }
    if (TeleOpEndTimerEnabled) {
      if (SmartDashboard.getNumber("TeleOpEndTimer", 0) <= .02) {
        TeleOpEndTimerEnabled = false;
      } else {
        SmartDashboard.putNumber(
            "TeleOpEndTimer", SmartDashboard.getNumber("TeleOpEndTimer", 0) - .02);
      }
    }
    if (AutoTimerEnabled) {
      if (SmartDashboard.getNumber("AutoTimer", 0) <= .02) {
        AutoTimerEnabled = false;
      } else {
        SmartDashboard.putNumber("AutoTimer", SmartDashboard.getNumber("AutoTimer", 0) - .02);
      }
    }
    SmartDashboard.updateValues();
  }

  // check enabled if enabled
  public static boolean isTeleOpStartTimerEnabled() {
    return TeleOpStartTimerEnabled;
  }

  public static boolean isTeleOpEndTimerEnabled() {
    return TeleOpEndTimerEnabled;
  }

  public static boolean isAutoTimerEnabled() {
    return AutoTimerEnabled;
  }

  // get timer values
  public static double getTeleOpStartTimerValue() {
    return SmartDashboard.getNumber("TeleOpTimerStart", 0);
  }

  public static double getTeleOpEndTimerValue() {
    return SmartDashboard.getNumber("TeleOpTimerEnd", 0);
  }

  public static double getAutoTimerValue() {
    return SmartDashboard.getNumber("AutoTimer", 0);
  }

  // set status timers status
  public static void setAutoTimerEnabled() {
    AutoTimerEnabled = true;
  }

  public static void setTeleOpStartTimerEnabled() {
    TeleOpStartTimerEnabled = true;
  }

  public static void setTeleOpEndTimerEnabled() {
    TeleOpEndTimerEnabled = true;
  }

  public static void setAutoTimerDisabled() {
    AutoTimerEnabled = false;
  }

  public static void setTeleOpStartTimerDisabled() {
    TeleOpStartTimerEnabled = false;
  }

  public static void setTeleOpEndTimerDisabled() {
    TeleOpEndTimerEnabled = false;
  }
}
