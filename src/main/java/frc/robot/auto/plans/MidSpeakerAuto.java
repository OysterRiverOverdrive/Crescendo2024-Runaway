// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoFeederCmd;
import frc.robot.auto.AutoShooterCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.auto.AutoIntakeCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;

public class MidSpeakerAuto extends ParallelCommandGroup {
  public MidSpeakerAuto(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      FeederSubsystem feeder,
      ShooterSubsystem shooter) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands
    Command speakerShoot =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(0.2, 0)), new Pose2d(0.45, 0, new Rotation2d(0)));
    Command toNoteToSpeaker =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(1.4, 0)), new Pose2d(1.28, 0, new Rotation2d(0)));
    Command toMidNote = autodrive.AutoDriveCmd(drivetrain, List.of(new Translation2d(-0.2,0), new Translation2d()), new Pose2d(-1,0, new Rotation2d(0)));
    // Command speakerShoot2 =
    //     autodrive.AutoDriveCmd(
    //         drivetrain, List.of(new Translation2d(0.2, 0)), new Pose2d(0.45, 0, new Rotation2d(0)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Drivetrain Sequential
        new SequentialCommandGroup(speakerShoot, new AutoSleepCmd(4), toNoteToSpeaker, toMidNote),

        // Intake Sequential
        new SequentialCommandGroup(new AutoSleepCmd(5), new AutoIntakeCmd(intake, 2.5)),

        // Feeder Sequential
        new SequentialCommandGroup(new AutoSleepCmd(2.5), new AutoFeederCmd(feeder, true, 0.5), new AutoSleepCmd(0.5), new AutoFeederCmd(feeder, true, 4)),

        // Shooter Sequential
        new SequentialCommandGroup(new AutoShooterCmd(shooter, 1, 1, 3), new AutoSleepCmd(5), new AutoShooterCmd(shooter, 1, 1, 6))
        );
  }
}
