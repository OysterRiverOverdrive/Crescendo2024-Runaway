// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_BMotor;
  private CANSparkMax m_RMotor;

  public IntakeSubsystem() {

    m_BMotor = new CANSparkMax(Constants.BumperMotor, MotorType.kBrushless);
    m_RMotor = new CANSparkMax(Constants.RollerMotor, MotorType.kBrushless);
  }

  public void BmotorF() {
    m_BMotor.set(Constants.RobotConstants.intakeMotorForward);
  }

  public void BmotorB() {
    m_BMotor.set(Constants.RobotConstants.intakeMotorBackward);
  }

  public void BmotorStop() {
    m_BMotor.stopMotor();
  }

  public void RmotorF() {
    m_RMotor.set(Constants.RobotConstants.intakeMotorForward);
  }

  public void RmotorB() {
    m_RMotor.set(Constants.RobotConstants.intakeMotorBackward);
  }

  public void RmotorStop() {
    m_RMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
