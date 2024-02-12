// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
// Import Intake Subsystem

// Reference: https://github.com/OysterRiverOverdrive/Crescendo2024-Runaway/blob/IntakeColorCindy/src/main/java/frc/robot/commands/InFeederCmd.java
public class OuttakeCmd extends Command {
  // Instaniate Intake Subsystem with empty value

  public OuttakeCmd(/*Add intake subsystem value */) {
    // Set the empty value equal to value provided in line 14
    // add requirements see reference
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Call methods created in subsystem
    // set rollers to spin out
    // set bumper wheel to spin out
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
