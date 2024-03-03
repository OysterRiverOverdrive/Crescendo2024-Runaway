// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ExampleAuton extends ParallelCommandGroup {
  public ExampleAuton(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, FeederSubsystem feeder, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Drivetrain Sequential
      new SequentialCommandGroup(null),
      // Intake Sequential
      new SequentialCommandGroup(null),
      // Feeder Sequential
      new SequentialCommandGroup(null),
      // Shooter Sequential
      new SequentialCommandGroup(null),
    );
  }
}
