// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Controllers;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterForwardCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;

  private final Joystick oper = new Joystick(DriveConstants.kOperControllerPort);

  public ShooterForwardCmd(ShooterSubsystem shooters) {
    shooter = shooters;
    addRequirements(shooters);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // trigger value (how far it's pushed in) is set as the speed of the motor
    double trigValue = oper.getRawAxis(Controllers.ps4_RT);
    shooter.ShooterForwardCmd(trigValue);
    
    // If the trigger is pushed in enough spin out the amp arm
    if (trigValue >= 0.2) {
      shooter.AmpArmUpCmd();
    } else {
      shooter.AmpArmDownCmd();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
