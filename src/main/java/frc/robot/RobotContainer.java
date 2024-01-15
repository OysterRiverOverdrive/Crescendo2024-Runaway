// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopCmd;
import frc.robot.commands.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  // Commands
  private final AutoCreationCmd autodrive = new AutoCreationCmd();
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
    // return null;
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    return autodrive.AutoDriveCmd(
        drivetrain,
        List.of(new Translation2d(0, 1), new Translation2d(1, 1), new Translation2d(1, 0)),
        new Pose2d(0, 0, new Rotation2d(-180)));
  }
}
