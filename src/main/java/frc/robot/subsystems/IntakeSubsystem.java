// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Instanciate two motors
  // One runs the bumper wheel, its brushless
  // The other runs the rollers, also brushless

  public IntakeSubsystem() {}

  // Create a Method to run Bumper Wheel forward call it "setBumperWheelForward", speeds in Robot Constants
  
  // Create a Method to run Bumper Wheel reverse call it "setBumperWheelReverse", speeds in Robot Constants

  // Create a Method to stop Bumper Wheel call it "bumperWheelStop", speeds in Robot Constants

  // Create a Method to run Roller forward call it "setRollerForward", speeds in Robot Constants
  
  // Create a Method to run Roller reverse call it "setRollerReverse", speeds in Robot Constants

  // Create a Method to stop Roller call it "rollerStop", speeds in Robot Constants

  @Override
  public void periodic() {
    
  }
}
