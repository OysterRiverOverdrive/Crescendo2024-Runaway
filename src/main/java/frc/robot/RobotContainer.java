// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
// import java.util.List;

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
    return null;
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    //   // 1. Create trajectory settings
    //   TrajectoryConfig trajectoryConfig =
    //       new TrajectoryConfig(
    //               AutoConstants.kMaxSpeedMetersPerSecond,
    //               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //           .setKinematics(DriveConstants.kDriveKinematics);

    //   // 2. Generate trajectory
    //   Trajectory trajectory =
    //       TrajectoryGenerator.generateTrajectory(
    //           new Pose2d(0, 0, new Rotation2d(0)),
    //           List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
    //           new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
    //           trajectoryConfig);

    //   // 3. Define PID controllers for tracking trajectory
    //   // PIDController xController =
    //   // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    //   ProfiledPIDController thetaController =
    //       new ProfiledPIDController(
    //           AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //   // 4. Construct command to follow trajectory
    //   SwerveControllerCommand swerveControllerCommand =
    //       new SwerveControllerCommand(
    //           trajectory,
    //           drivetrain::getPose,
    //           DriveConstants.kDriveKinematics,
    //           new PIDController(AutoConstants.kPXController, 0, 0),
    //           new PIDController(AutoConstants.kPYController, 0, 0),
    //           thetaController,
    //           drivetrain::setModuleStates,
    //           drivetrain);

    //   // 5. Add some init and wrap-up, and return everything
    //   return new SequentialCommandGroup(
    //       new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
    //       swerveControllerCommand,
    //       new InstantCommand(() -> drivetrain.stopModules()));
  }
}
