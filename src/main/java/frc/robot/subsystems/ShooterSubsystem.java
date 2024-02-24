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
  private CANSparkMax m_ampArmMotor =
      new CANSparkMax(RobotConstants.kAmpArmCanId, MotorType.kBrushless);
  private RelativeEncoder encAmpArm = m_ampArmMotor.getEncoder();

  public ShooterSubsystem() {
    m_followMotor.follow(m_leadMotor, true);
    encAmpArm.setPosition(0); // Reset Encoder on Boot
  }

  public void ShooterForwardCmd(double trigValue) {
    m_leadMotor.set(trigValue);
    System.out.println(trigValue);
  }

  public void motorStop() {
    m_leadMotor.stopMotor();
  }

  public double getAmpArmEnc() {
    return encAmpArm.getPosition()
        / RobotConstants
            .kAmpArmGearRatio; // Divide for the gear ratio to get the position of the arm, not the
    // motor
  }

  public void setArmSpeed(double speed) {
    m_ampArmMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("amp encoder: " + ampEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
