// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;
  private final Joystick operator = new Joystick(Controllers.OPER_PORT);

  public ShooterSubsystem() {

    m_leadMotor = new CANSparkMax(9, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(10, MotorType.kBrushless);
    m_followMotor.follow(m_leadMotor);

  }

  public void motorF() {
    //set speed equal to value of trigger
    double speed = operator.getRawAxis(Controllers.logi_rt)/2;
    m_leadMotor.set(speed);
    System.out.println(speed);
  }

  public void motorStop() {
    m_leadMotor.stopMotor();
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
