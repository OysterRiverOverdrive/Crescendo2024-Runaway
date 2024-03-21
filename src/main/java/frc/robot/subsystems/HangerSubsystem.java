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

  private boolean Compressor = true;

  public HangerSubsystem() {
    // If Compressor on Bot
    m_compressor.enableAnalog(105, 120);
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
      Compressor = false;
      m_compressor.enableAnalog(85, 120);
    } else if (!m_compressor.isEnabled()) {
      Compressor = true;
      m_compressor.enableAnalog(110, 120);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pneumatics Pressure (PSI)", getPressure());
    SmartDashboard.putBoolean("Compressor Fill Mode", Compressor);
  }
}
