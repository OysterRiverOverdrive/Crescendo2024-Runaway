// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_leadMotor = new CANSparkMax(RobotConstants.kShooterLeftCanId, MotorType.kBrushless);
  private CANSparkMax m_followMotor = new CANSparkMax(RobotConstants.kShooterRightCanId, MotorType.kBrushless);

  public ShooterSubsystem() {
    m_followMotor.follow(m_leadMotor, true);
  }

  public void ShooterForwardCmd(double trigValue) {
    m_leadMotor.set(trigValue);
    System.out.println(trigValue);
  }

  public void motorStop() {
    m_leadMotor.stopMotor();
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
