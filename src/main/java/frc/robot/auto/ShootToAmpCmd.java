package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class ShootToAmpCmd extends ParallelCommandGroup {
  public ShootToAmpCmd(
      DrivetrainSubsystem drive,
      IntakeSubsystem intake,
      FeederSubsystem feeder,
      ShooterSubsystem shooter) {

    addCommands(null);
  }
}
