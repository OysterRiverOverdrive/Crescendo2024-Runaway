// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_leadMotor =
      new CANSparkMax(RobotConstants.kShooterLeftCanId, MotorType.kBrushless);
  private CANSparkMax m_followMotor =
      new CANSparkMax(RobotConstants.kShooterRightCanId, MotorType.kBrushless);
  private CANSparkMax ampMotor = new CANSparkMax(RobotConstants.kAmpArmCanId, MotorType.kBrushless);

  RelativeEncoder ampEncoder = ampMotor.getEncoder();

  SparkPIDController ampMotorPidController = ampMotor.getPIDController();

  public ShooterSubsystem() {
    m_followMotor.follow(m_leadMotor, true);

    ampMotor.setIdleMode(IdleMode.kBrake);
    ampMotorPidController.setFeedbackDevice(ampEncoder);
    ampMotorPidController.setP(0.5);
    ampMotorPidController.setI(0);
    ampMotorPidController.setD(0);
  }

  public void ShooterForwardCmd(double trigValue) {
    m_leadMotor.set(trigValue);
    System.out.println(trigValue);
  }

  public void motorStop() {
    m_leadMotor.stopMotor();
  }

  public void AmpArmUpCmd() {
    // Degrees from resting position (May Need to add a negative)
    double degrees = 100;
    // Convert account for gear ratio
    degrees = degrees * RobotConstants.kAmpArmGearRatio;
    // Convert to radians
    degrees = degrees * (Math.PI / 180);
    // Set position
    ampMotorPidController.setReference(degrees, CANSparkMax.ControlType.kPosition);
  }

  public void AmpArmDownCmd() {
    ampMotorPidController.setReference(0, CANSparkMax.ControlType.kPosition);
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
