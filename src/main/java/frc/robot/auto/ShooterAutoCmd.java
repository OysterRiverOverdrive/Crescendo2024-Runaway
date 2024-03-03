// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutoCmd extends Command {
  private Timer timer = new Timer();
  ShooterSubsystem shoot;
  double inSpeed;
  double timeRunning;

  public ShooterAutoCmd(ShooterSubsystem shoots, double inputSpeed, double timeRun) {
    shoot = shoots;
    inSpeed = inputSpeed;
    timeRunning = timeRun;
    addRequirements(shoots);
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
    shoot.ShooterForwardCmd(inSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.motorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return timer.hasElapsed(timeRunning) ? true : false;
  }
}
