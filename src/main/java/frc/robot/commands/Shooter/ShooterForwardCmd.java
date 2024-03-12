// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Controllers;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterForwardCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooter;

  private final Joystick oper = new Joystick(DriveConstants.kOperControllerPort);
  private final PIDController pidControl =
      new PIDController(RobotConstants.kAmpArmP, RobotConstants.kAmpArmI, RobotConstants.kAmpArmD);

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
    shooter.ShooterForwardCmd(trigValue * 0.6, trigValue * 0.6);

    double degreeout = RobotConstants.kAmpArmDegreesOut / 360; // Convert to percentage of rotation
    if (trigValue >= RobotConstants.kAmpArmTrigActivate) {
      shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), degreeout));
    } else {
      shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), 0));
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
