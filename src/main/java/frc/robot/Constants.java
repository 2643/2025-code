// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final int BottomHard = -10000;
  public static final int TopHard = 10000;
  public static final int BottomSoft = -3000;
  public static final int TopSoft = 2000;
  
  public static final double L1L2L3 = -0.238701171875;
  public static final double L4 = 2;
  public static final double GROUND = 3;
  public static final double FEEDER = 4;
  public static final double BARGE = 5;
  public static final double REST = 0;

  public static final double GRABBERGEARRATIO = 100;
  
  public static final double CURRENTLIMIT = 11;
}
