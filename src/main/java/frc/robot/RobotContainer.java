// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StationaryCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.InwardCommand;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopCmd;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.DrivetrainSubsystem;
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
    supplier(1).onTrue(forward).onFalse(stop);
    supplier(2).onTrue(backward).onFalse(stop);
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java

    controllerutil
        .supplier(Controllers.logi_b, DriveConstants.joysticks.DRIVER)
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
    return new SequentialCommandGroup(auto);
  }

  private final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();

  private final Joystick XBoxController = new Joystick(0);

  private final InwardCommand forward = new InwardCommand(m_intakesubsystem);
  private final EjectCommand backward = new EjectCommand(m_intakesubsystem);
  private final StationaryCommand stop = new StationaryCommand(m_intakesubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public Trigger supplier(int buttonID) {
    BooleanSupplier bsup = () -> XBoxController.getRawButton(buttonID);
    Trigger mybutton = new Trigger(bsup);
    return mybutton;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
}
