// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopCmd;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.utils.ControllerUtils;
import java.util.List;

public class RobotContainer {
  // Auto Dropdown - Make dropdown variable and variables to be selected
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String auto1 = "1";
  private final String auto2 = "2";
  private final String auto3 = "3";
  private final String auto4 = "4";

  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ControllerUtils controllerutil = new ControllerUtils();

  // Commands
  private final AutoCreationCmd autodrive = new AutoCreationCmd();
  private final TeleopCmd teleopCmd = new TeleopCmd(drivetrain);

  // Auto Driving Commands
  // Drive in a circle (Diameter: 1 Meter)
  private final Command driveCircle =
      autodrive.AutoDriveCmd(
          drivetrain,
          List.of(
              new Translation2d(0, 1),
              new Translation2d(2, 1),
              new Translation2d(2, -1),
              new Translation2d(4, -1),
              new Translation2d(4, 1),
              new Translation2d(2, 1),
              new Translation2d(2, -1),
              new Translation2d(0, -1)),
          new Pose2d(0, 0, new Rotation2d(0)));

  public RobotContainer() {
    // Declare default command during Teleop Period as TeleopCmd(Driving Command)
    drivetrain.setDefaultCommand(teleopCmd);

    // Add Auto options to dropdown and push to dashboard
    m_chooser.setDefaultOption("Circle", auto1);
    m_chooser.addOption("Null1", auto2);
    m_chooser.addOption("Null2", auto3);
    m_chooser.addOption("Null3", auto4);
    SmartDashboard.putData("Auto Selector", m_chooser);
    SmartDashboard.putNumber("Auto Wait Time (Sec)", 0);

    // Configure Buttons Methods
    configureBindings();
  }

  private void configureBindings() {
    // Configure buttons
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java

    controllerutil
        .supplier(6, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
  }

  public Command getAutonomousCommand() {

    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    Command auto;
    switch (m_chooser.getSelected()) {
      case auto1:
      default:
        auto = driveCircle;
      case auto2:
        auto = null;
      case auto3:
        auto = null;
      case auto4:
        auto = null;
    }
    return auto;
  }
}
