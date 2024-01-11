// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopCmd;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  // Commands
  private final TeleopCmd teleopCmd = new TeleopCmd(drivetrain);

  public RobotContainer() {
    // Declare default command during Teleop Period as TeleopCmd(Driving Command)
    drivetrain.setDefaultCommand(teleopCmd);

    // Configure Buttons Methods
    configureBindings();
  }

  private void configureBindings() {
    // Configure buttons
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // Return NOTHING, replace with command to be run in autonomous period
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    return null;
  }
}
