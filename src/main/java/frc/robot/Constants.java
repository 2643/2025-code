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
  public static double BottomSoft = -10;
  public static double BottomHard = -20;
  public static double TopSoft = 20;
  public static double TopHard = 30;
  public static double L1 = 1;
  public static double L2 = 2;
  public static double L3 = 3;
  public static double L4 = 4;
  public static double groundLevel = 5;
  public static double feederLevel = 6;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
