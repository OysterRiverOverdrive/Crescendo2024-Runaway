// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class AutoFeederCmd extends Command {
  private FeederSubsystem feeder;
  private Timer timer = new Timer();
  public boolean toShooter;
  public double timeNum;

  public AutoFeederCmd(FeederSubsystem feeders, boolean goToShooter, double time) {
    feeder = feeders;
    toShooter = goToShooter;
    timeNum = time;
    addRequirements(feeders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (toShooter) {
      feeder.ToShooterCmd();
    } else {
      feeder.InFeederCmd();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.StopFeederCmd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTime = timer.get();

    return currTime >= timeNum ? true : false;
  }
}
