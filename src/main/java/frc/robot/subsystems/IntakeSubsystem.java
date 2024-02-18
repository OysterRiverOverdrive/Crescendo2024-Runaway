// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_BumperMotor =
      new CANSparkMax(RobotConstants.BumperMotorCanId, MotorType.kBrushless);
  private CANSparkMax m_RollerMotor =
      new CANSparkMax(RobotConstants.RollerMotorCanId, MotorType.kBrushless);

  public IntakeSubsystem() {}

  public void BmotorF() {
    m_BumperMotor.set(RobotConstants.bumperMotorForward);
  }

  public void BmotorB() {
    m_BumperMotor.set(RobotConstants.bumperMotorBackward);
  }

  public void BmotorStop() {
    m_BumperMotor.stopMotor();
  }

  public void RmotorF() {
    m_RollerMotor.set(RobotConstants.rollerMotorForward);
  }

  public void RmotorB() {
    m_RollerMotor.set(RobotConstants.rollerMotorBackward);
  }

  public void RmotorStop() {
    m_RollerMotor.stopMotor();
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
