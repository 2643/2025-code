// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  double x;
  double y;
  double autoP = 0.05;
  double autoI = 0.005;
  double autoD = 0;
  double[] botPoseBlueTable;
  final double maxPos = -0.205;
  final double minPos = -0.15;
  final double maxDis = -4;
  final double minDis = 21;
  double armPercent;
  double offset = Units.degreesToRadians(98);

  double yDisReefL1 = Units.inchesToMeters(66.5);
  double yDisReefL2 = Units.inchesToMeters(66.5);
  double yDisReefL3 = Units.inchesToMeters(66.5);
  double yDisReefL4 = Units.inchesToMeters(66.5);

  GenericEntry angleOffset = Shuffleboard.getTab("autoPID").add("angle", 100).getEntry();
  GenericEntry errorEntry = Shuffleboard.getTab("autoPID").add("Error", 0).getEntry();
  GenericEntry aprilEntry = Shuffleboard.getTab("autoPID").add("Apriltag", false).getEntry();

  LinearFilter autoAimFilter = LinearFilter.movingAverage(10);
  MedianFilter outlierFilter = new MedianFilter(7);

  PIDController autoAnglePID = new PIDController(0.01, 0, 0);
  PIDController angularLockPID = new PIDController(0.05, 0.0005, 0.04);

  /** Creates a new Vision. */

  public Vision() {

  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-smokey");
    NetworkTableEntry botPoseBlue = table.getEntry("botpose_wpiblue");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    botPoseBlueTable = botPoseBlue.getDoubleArray(new double[11]);

    errorEntry.setDouble(x);
    aprilEntry.setBoolean(isApriltag());

    // This method will be called once per scheduler run
  }

  public boolean isApriltag() {
    if (x == 0 && y == 0)
      return false;
    else
      return true;
  }

  public void autoAngle() {

  }

}
