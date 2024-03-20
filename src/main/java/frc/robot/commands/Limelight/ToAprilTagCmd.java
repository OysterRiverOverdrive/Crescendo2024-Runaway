// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ToAprilTagCmd extends Command{
    private LimelightSubsystem limelight;

    public ToAprilTagCmd(LimelightSubsystem limelights) {
        limelight = limelights;
        addRequirements(limelights);
    }
}
