// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;

  public IntakeSubsystem() {

    m_leadMotor = new CANSparkMax(Constants.UnderMotor, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(Constants.TopMotor, MotorType.kBrushless);
    m_followMotor.follow(m_leadMotor);
  }

  public void motorF() {
    m_leadMotor.set(0.8);
  }

  public void motorB() {
    m_leadMotor.set(-0.8);
  }

  public void motorStop() {
    m_leadMotor.stopMotor();
  }

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
