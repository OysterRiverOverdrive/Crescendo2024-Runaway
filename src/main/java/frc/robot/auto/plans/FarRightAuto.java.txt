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

public class FarRightAuto extends ParallelCommandGroup {

  public FarRightAuto(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      FeederSubsystem feeder,
      ShooterSubsystem shooter) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command driveAuto1 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(.3, .05)),
            new Pose2d(.6, .1, new Rotation2d(2 / 3 * Math.PI)));
    Command driveAuto2 = 
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(6.6074/2,3.3148/2)), 
            new Pose2d(6.6074,3.3148,new Rotation2d(-2/3 * Math.PI)));
    Command driveAuto3 = 
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(0,-7.6296/2)), 
            new Pose2d(0,-7.6296,new Rotation2d(0)));
    addCommands(

        // Driving groups
        new SequentialCommandGroup(driveAuto1, new AutoSleepCmd(1), driveAuto2,new AutoSleepCmd(2), driveAuto3),

        // Intake group
        new SequentialCommandGroup(new AutoSleepCmd(5), new IntakeAutoCmd(intake, 5)),

        // Feeder group
        new SequentialCommandGroup(
            new AutoSleepCmd(1), new AutoFeederCmd(feeder, true, .5),
            new AutoSleepCmd(10), new AutoFeederCmd(feeder, true, .5)),
        // Shooter group
        new SequentialCommandGroup(
            new AutoSleepCmd(.5), new AutoShooterCmd(shooter, 1, .8, 1),
            new AutoSleepCmd(8), new AutoShooterCmd(shooter, 1, .8, 1)));
  }
}
