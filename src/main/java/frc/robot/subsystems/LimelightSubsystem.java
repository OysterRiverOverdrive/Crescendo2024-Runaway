// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.VisionConstants;

public class LimelightSubsystem extends SubsystemBase {
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry botpose;
    private NetworkTableEntry tid;

  /** Creates a new LimelightSubSys. */
  public LimelightSubsystem() {

    // TODO: do we need pipeline here?
     table = NetworkTableInstance.getDefault().getTable("limelight");
     tx = table.getEntry("tx");
     ty = table.getEntry("ty");
     ta = table.getEntry("ta");
     botpose = table.getEntry("botpose");
     tid = table.getEntry("tid"); 
  }

  @Override
  public void periodic() {
  //read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double targetid = tid.getDouble(0.0);
  double[] fieldpose = botpose.getDoubleArray(new double[8]);

  //TODO: change to SmartDashboard
  System.out.println("LimelightX:" + x );
  System.out.println("LimelightY:" + y);
  System.out.println("LimelightArea:" + area);
  System.out.println("Target ID:" + targetid);
  System.out.println("Field pose X:" + fieldpose[0]);
  System.out.println("Field pose Y:" + fieldpose[1]);
  System.out.println("Field pose Z:" + fieldpose[2]);
  System.out.println("Field pose Roll:" + fieldpose[3]);
  System.out.println("Field pose Pitch:" + fieldpose[4]);
  System.out.println("Field pose Yaw:" + fieldpose[5]);
  }

}