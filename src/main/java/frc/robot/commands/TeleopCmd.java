// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleopCmd extends Command {
  /** Creates a new TeleopCmd. */
  private final DrivetrainSubsystem driveSub;
  // Create a controller object
  private final Joystick controller = new Joystick(DriveConstants.kDrveControllerPort);

  public TeleopCmd(DrivetrainSubsystem drives) {
    driveSub = drives;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double deadzone(double joyvalue) {
    if (Math.abs(joyvalue) > DriveConstants.deadzoneDriver) {
      return joyvalue;
    } else {
      return 0.0;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ContX = deadzone(controller.getRawAxis(DriveConstants.kDriveX)) * -1;
    double ContY = deadzone(controller.getRawAxis(DriveConstants.kDriveY)) * -1;
    double ContRotate = deadzone(controller.getRawAxis(DriveConstants.kDriveRotate)) * -1;
    driveSub.drive(ContX, ContY, ContRotate, false, false);
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
