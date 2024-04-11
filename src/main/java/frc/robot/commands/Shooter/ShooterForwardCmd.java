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
import java.util.function.Supplier;

public class ShooterForwardCmd extends Command {
  private final ShooterSubsystem shooter;
  private final Joystick oper = new Joystick(DriveConstants.kOperControllerPort);
  private final PIDController pidControl =
      new PIDController(RobotConstants.kAmpArmP, RobotConstants.kAmpArmI, RobotConstants.kAmpArmD);
  private final Supplier<Boolean> fullShot;
  private final Supplier<Boolean> leftShot;
  private final Supplier<Boolean> rightShot;

  public ShooterForwardCmd(
      ShooterSubsystem shooters,
      Supplier<Boolean> fullShotSupplier,
      Supplier<Boolean> leftShotSupplier,
      Supplier<Boolean> rightShotSupplier) {
    fullShot = fullShotSupplier;
    leftShot = leftShotSupplier;
    rightShot = rightShotSupplier;
    shooter = shooters;
    addRequirements(shooters);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (fullShot.get()) {
      shooter.ShooterForwardCmd(1, 1);
      shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), 0));
    } else if (leftShot.get()) {
      shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), 0));
      shooter.ShooterForwardCmd(0.9, 1);
    } else if (rightShot.get()) {
      shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), 0));
      shooter.ShooterForwardCmd(1, 0.9);
    } else {
      // trigger value (how far it's pushed in) is set as the speed of the motor
      double trigValue = oper.getRawAxis(Controllers.ps4_RT);
      shooter.ShooterForwardCmd(trigValue * 0.5, trigValue * 0.5);

      // Amp arm PID and activation
      double degreeout =
          RobotConstants.kAmpArmDegreesOut / 360; // Convert to percentage of rotation
      if (trigValue >= RobotConstants.kAmpArmTrigActivate) {
        shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), degreeout));
      } else {
        shooter.setArmSpeed(pidControl.calculate(shooter.getAmpArmEnc(), 0));
      }
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
