// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx = table.getEntry("tx"); // x coordinate of tag in camera image
  private final NetworkTableEntry ty = table.getEntry("ty"); // y coordinate of tag in camera image
  private final NetworkTableEntry ta = table.getEntry("ta"); // area of tag in camera image
  private final NetworkTableEntry botpose =
      table.getEntry(
          "botpose"); // bot pose (x, y, z, roll, pitch, yaw, total latency (not used currently))
  private final NetworkTableEntry botpose_wpired = table.getEntry("botpose_wpired");
  private final NetworkTableEntry botpose_wpiblue = table.getEntry("botpose_wpiblue");
  private final NetworkTableEntry tid = table.getEntry("tid"); // ID of currently-seen target
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String abs_choice = "absolute coodinates";
  private final String alliance_choice = "alliance coordinates";
  private boolean absoluteCoordinates;

  /** Creates a new LimelightSubSys. */
  public LimelightSubsystem() {
    // default to absolute coordinates, with (0,0) at field center
    setAbsoluteCoords();

    m_chooser.setDefaultOption("Absolute", abs_choice);
    m_chooser.addOption("Alliance", alliance_choice);
    SmartDashboard.putData("Limelight Coordinates", m_chooser);
  }

  @Override
  public void periodic() {
    if (m_chooser.getSelected().equals(abs_choice)) {
      setAbsoluteCoords();
    } else {
      setAllianceCoords();
    }

    // read values periodically
    double x = getAprilTagX();
    double y = getAprilTagY();
    double area = getAprilTagArea();
    int targetid = getAprilTagID();
    // call either getAbsoluteBotPose or getTeamBotPose
    double[] fieldpose;
    if (absoluteCoordinates) {
      fieldpose = getAbsoluteBotPose();
    } else {
      fieldpose = getAllianceBotPose();
    }

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target ID", targetid);
    SmartDashboard.putNumber("Field pose X", fieldpose[0]);
    SmartDashboard.putNumber("Field pose Y", fieldpose[1]);
    SmartDashboard.putNumber("Field pose Z", fieldpose[2]);
    SmartDashboard.putNumber("Field pose Roll", fieldpose[3]);
    SmartDashboard.putNumber("Field pose Pitch", fieldpose[4]);
    SmartDashboard.putNumber("Field pose Yaw", fieldpose[5]);
  }

  public void setAbsoluteCoords() {
    absoluteCoordinates = true;
  }

  public void setAllianceCoords() {
    absoluteCoordinates = false;
  }

  public double[] getAbsoluteBotPose() {
    return botpose.getDoubleArray(new double[6]);
  }

  public double[] getAllianceBotPose() {
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
      return botpose_wpiblue.getDoubleArray(new double[6]);
    } else {
      return botpose_wpired.getDoubleArray(new double[6]);
    }
  }

  public int getAprilTagID() {
    return (int) tid.getDouble(0.0);
  }

  public double getAprilTagX() {
    return tx.getDouble(0.0);
  }

  public double getAprilTagY() {
    return ty.getDouble(0.0);
  }

  public double getAprilTagArea() {
    return ta.getDouble(0.0);
  }

  // Set the pose of the camera relative to the robot. Can also be set in web interface
  public void setCameraposeRobotspace(double[] camPose) {
    table.getEntry("camerapose_robotspace_set").setDoubleArray(camPose);
  }
}