// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_shooterMotor1 =
      new CANSparkMax(RobotConstants.kShooterLeftCanId, MotorType.kBrushless);
  private CANSparkMax m_shooterMotor2 =
      new CANSparkMax(RobotConstants.kShooterRightCanId, MotorType.kBrushless);

  public ShooterSubsystem() {
    m_shooterMotor2.setInverted(true);
  }

  public void ShooterForwardCmd(double trigValue) {
    m_shooterMotor1.set(trigValue);
    m_shooterMotor2.set(0.85 * trigValue);
  }

  public void motorStop() {
    m_shooterMotor1.stopMotor();
    m_shooterMotor2.stopMotor();
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
