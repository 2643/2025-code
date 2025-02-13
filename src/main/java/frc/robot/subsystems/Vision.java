// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
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
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  double[] botPoseBlueTable;
  double targetX;
  double targetY;
  double targetZ;
  double ta;
  double area;
  double x;
  double y;
  double targetarea;
  double autoP = 0.05;
  double autoI = 0.005;
  double autoD = 0;
  static double rz;
  final double maxPos = -0.205;
  final double minPos = -0.15;
  final double maxDis = -4;
  final double minDis = 21;
  boolean checker = false;
  double armPercent;
  double offset = Units.degreesToRadians(98);
  double trig;
  double yDisShooterSpeaker = Units.inchesToMeters(66.5);


  // GenericEntry autoPEntry = Shuffleboard.getTab("autoPID").add("P",
  // 0).getEntry();
  // GenericEntry autoIEntry = Shuffleboard.getTab("autoPID").add("I",
  // 0).getEntry();
  // GenericEntry autoDEntry = Shuffleboard.getTab("autoPID").add("D",
  // 0).getEntry();
  GenericEntry angleOffset = Shuffleboard.getTab("autoPID").add("angle", 100).getEntry();
  GenericEntry errorEntry = Shuffleboard.getTab("autoPID").add("Error", 0).getEntry();
  GenericEntry aprilEntry = Shuffleboard.getTab("autoPID").add("Apriltag", false).getEntry();

  // GenericEntry turnEntry = Shuffleboard.getTab("autoPID").add("Turn",
  // 0).getEntry();
  LinearFilter autoAimFilter = LinearFilter.movingAverage(10);
  MedianFilter outlierFilter = new MedianFilter(7);

  PIDController autoAnglePID = new PIDController(0.0035, 0, 0.000001);
  PIDController angularLockPID = new PIDController(0.05, 0.0005, 0.04);

  GenericEntry autoanglep = Shuffleboard.getTab("autoPID").add("kp", 0.0015).getEntry();
  GenericEntry autoanglei = Shuffleboard.getTab("autoPID").add("ki", 0.0).getEntry();
  GenericEntry autoangled = Shuffleboard.getTab("autoPID").add("kd", 0.000001).getEntry();

  /** Creates a new Vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    // offset = Units.degreesToRadians(angleOffset.getDouble(100));
    autoAnglePID.setP(autoanglep.getDouble(0.0015));
    autoAnglePID.setI(autoanglei.getDouble(0.0));
    autoAnglePID.setD(autoangled.getDouble(0.000001));

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-smokey");
    NetworkTableEntry targetpose_cameraspace = table.getEntry("targetpose_cameraspace");
    double[] targetpose_cameraspaceArray = targetpose_cameraspace.getDoubleArray(new double[5]);
    
   
    rz = roundAvoid(targetpose_cameraspaceArray[4], 2);

    SmartDashboard.putNumber("rz", rz);

    // autoAnglePID.setP(autoPEntry.getDouble(0));
    // autoAnglePID.setI(autoIEntry.getDouble(0));
    // autoAnglePID.setD(autoDEntry.getDouble(0));

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    x =  tx.getDouble(0);
    y =  ty.getDouble(0);
    area = ta.getDouble(0);
    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("area", area);
    // double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    errorEntry.setDouble(x);
    aprilEntry.setBoolean(isApriltag());
    // This method will be called once per scheduler run
  }

  public static double roundAvoid(double value, int places) {
    double scale = Math.pow(10, places);
    return Math.round(value * scale) / scale;
  }
  // needs to be readjusted

  public double autostrafe() {
    targetX = x;
    checker = false;
    if (!RobotContainer.centerorLeftRight.getAsBoolean()) {
      if (RobotContainer.LeftRight.getAsBoolean()) {
        if (checker) {
          targetX = x + 26;
        }
        else {
          targetX = x + 16;
        }
        checker = true;
      } else {
        if (checker) {
          targetX = x - 21;
        }
        else {
          targetX = x - 11;
        }
        checker = true;
      }
    } 
    else {
      targetX = x+6;
      checker = false;
    }
    
    // turnEntry.setDouble(autoAnglePID.calculate(targetX) - 0.1);
    return -autoAnglePID.calculate(targetX);

    // else
    // //turnEntry.setDouble(autoAnglePID.calculate(targetX + 2));
    // return autoAnglePID.calculate(targetX + 0.2);

  }

  public double autoAngle() {
    targetZ = rz;
    if (targetZ < 1 && targetZ > -1)
      targetZ = 0;
    return autoAnglePID.calculate(targetZ);

  }

  public double autotrans() {
    targetarea= area;
    return -autoAnglePID.calculate(targetarea,5.39);
  }
  // else
  // //turnEntry.setDouble(autoAnglePID.calculate(targetX + 2));
  // return autoAnglePID.calculate(targetX + 0.2);

  public double getTagPose() {
    System.out.println(angularLockPID.calculate(botPoseBlueTable[5]));
    return angularLockPID.calculate(botPoseBlueTable[5]);
  }

  public boolean isApriltag() {
    if (x == 0 && y == 0)
      return false;
    else
      return true;
  }

  public double getError() {
    return targetX;
  }
}