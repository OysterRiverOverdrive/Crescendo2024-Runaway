// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangerSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_rightSolenoid =
      new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);
  private final DoubleSolenoid m_leftSolenoid =
      new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 2, 3);
  private final Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  public HangerSubsystem() {
    // If Compressor on Bot
    // m_compressor.enableAnalog(min_pressure__psi, max_pressure_psi);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
