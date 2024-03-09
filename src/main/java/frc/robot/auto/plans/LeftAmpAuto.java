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
import frc.robot.auto.IntakeAutoCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;

public class LeftAmpAuto extends ParallelCommandGroup {
  public LeftAmpAuto(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      FeederSubsystem feeder,
      ShooterSubsystem shooter) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands
    Command ampShoot =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(0.2, 0)), new Pose2d(0.45, 0, new Rotation2d(0)));
    Command goToNote =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(0.5, 0), new Translation2d(1.2192,0), new Translation2d(1.2192,0.4)), new Pose2d(1.2192, 0.889, new Rotation2d(Math.PI/2)));
    Command ampShoot2 =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(1.2192,0.4), new Translation2d(1.2192,0), new Translation2d(0.5, 0)), new Pose2d(0.45, 0, new Rotation2d(-Math.PI/2)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Drivetrain Sequential
        new SequentialCommandGroup(ampShoot, new AutoSleepCmd(3.5), goToNote, new AutoSleepCmd(1),ampShoot2),

        // Intake Sequential
        new SequentialCommandGroup(new AutoSleepCmd(4.5), new IntakeAutoCmd(intake, true, 1)),

        // Feeder Sequential
        new SequentialCommandGroup(new AutoSleepCmd(2.5), new AutoFeederCmd(feeder, true, 0.5)),

        // Shooter Sequential
        new SequentialCommandGroup(new AutoShooterCmd(shooter, 1, 3)));
  }
}
