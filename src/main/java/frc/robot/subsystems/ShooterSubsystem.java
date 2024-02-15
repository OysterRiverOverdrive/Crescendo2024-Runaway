// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Controllers;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;
  private final Joystick operator = new Joystick(Controllers.OPER_PORT);

  public ShooterSubsystem() {

    m_leadMotor = new CANSparkMax(RobotConstants.kShooterCanId1, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(RobotConstants.kShooterCanId2, MotorType.kBrushless);
    m_followMotor.follow(m_leadMotor);
  }

  public void MotorForwardCmd() {
    //double speed = operator.getRawAxis(Controllers.xbox_rt) / 2;
    double speed = 0.4;
    m_leadMotor.set(speed);
    System.out.println(speed);
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
