// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class HangerSubsystem extends SubsystemBase {

  private final DoubleSolenoid m_rightSolenoid =
      new DoubleSolenoid(
          RobotConstants.kPneumaticHubCanId,
          PneumaticsModuleType.REVPH,
          RobotConstants.kHangerRightFwd,
          RobotConstants.kHangerRightBck);

  private final DoubleSolenoid m_leftSolenoid =
      new DoubleSolenoid(
          RobotConstants.kPneumaticHubCanId,
          PneumaticsModuleType.REVPH,
          RobotConstants.kHangerLeftFwd,
          RobotConstants.kHangerLeftBck);

  private final Compressor m_compressor =
      new Compressor(RobotConstants.kPneumaticHubCanId, PneumaticsModuleType.REVPH);

  private boolean prev_alert = false;

  public HangerSubsystem() {
    // If Compressor on Bot
    m_compressor.enableAnalog(75, 120);
  }

  public double getPressure() {
    return m_compressor.getPressure();
  }

  public void sendUpHanger() {
    m_leftSolenoid.set(DoubleSolenoid.Value.kForward);
    m_rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void sendDownHanger() {
    m_leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    m_rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void disableOrEnableCompressor() {
    if (m_compressor.isEnabled()) {
      m_compressor.disable();
    }
    else if(!m_compressor.isEnabled()){
      m_compressor.enableAnalog(75,120);
    }
  }

  public boolean checkRuns() {
    // A check to display on dashboard to show potential leaks
    double pressure = getPressure();
    if (pressure <= RobotConstants.kMinActuationPSI) {
      // Not Safe Alert - if statement to cause flickering on the dashboard for attention
      if (prev_alert) {
        prev_alert = true;
      } else {
        prev_alert = false;
      }
      return prev_alert;
    } else {
      // Safe
      prev_alert = true;
      return prev_alert;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pneumatics Pressure (PSI)", getPressure());
    SmartDashboard.putBoolean("Pneumatics Alert", checkRuns());
  }
}
