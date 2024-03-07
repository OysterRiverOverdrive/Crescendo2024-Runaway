// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoCmd extends Command {
  private Timer timer = new Timer();
  IntakeSubsystem intake;
  boolean bumperOrRollerMotor;
  double timeRunning;

  public IntakeAutoCmd(IntakeSubsystem intakes, boolean useBumperOrRollerMotor, double timeRun) {
    //True means only bumper motor will run, false means only roller motor will run.
    intake = intakes;
    bumperOrRollerMotor = useBumperOrRollerMotor;
    timeRunning = timeRun;
    addRequirements(intakes);
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
    if (bumperOrRollerMotor) {
        intake.BmotorF();
    } else {
        intake.RmotorF();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.BmotorStop();
    intake.RmotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return timer.hasElapsed(timeRunning) ? true : false;
  }
}

