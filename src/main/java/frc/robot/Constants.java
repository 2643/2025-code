package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;

public final class Constants {

  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  /**
   * The Constants class provides a convenient place for teams to hold robot-wide
   * numerical or boolean
   * constants. This class should not be used for any other purpose. All constants
   * should be declared
   * globally (i.e. public static). Do not put anything functional in this class.
   *
   * <p>
   * It is advised to statically import this class (or one of its inner classes)
   * wherever the
   * constants are needed, to reduce verbosity.
   */
  @SuppressWarnings({ "all" })

  public final class ClimbConstants {
    public static final int MOTOR_PORT = 0; // CHANGE THIS TO BE CORRECT!!
    public static final int LIMITSWITCH_PORT = 0;
    public static final double GEAR_RATIO = 350;

    public static final double BOTTOM_SOFT_LIMIT = -15;
    public static final double BOTTOM_HARD_LIMIT = -30;
    public static final double TOP_SOFT_LIMIT = 30;
    public static final double TOP_HARD_LIMIT = 33;

    public static final double CURRENT_LIMIT = 30;
    public static final double MOVE_TO_CAGE_POSITION = -0.335;
  }

  public final class ElevatorConstants {
    public static final int LEADER_MOTOR_PORT = 15;
    public static final int FOLLOWER_MOTOR_PORT = 14;
    public static final int LIMITSWITCH_PORT = 1;

    
    public static final double BOTTOM_SOFT_LIMIT =  -0.040;
    public static final double BOTTOM_HARD_LIMIT = -0.050;
    public static final double TOP_SOFT_LIMIT =  0.937;
    public static final double TOP_HARD_LIMIT = 0.943;

    // public static 

    public static final double LEVEL_1 = 0;
    public static final double LEVEL_2 = -0.265;
    public static final double LEVEL_3 = -0.52042-0.012;
    public static final double LEVEL_4 = -0.92;
    public static final double GROUND_POSITION = 0.019;
    public static final double FEEDER_POSITION = 0.019;
    public static final double BARGE_POSITION = -0.9124;
    public static final double REST_POSITION = 0;

    public static final double PROCESSOR_POSITION = 0;
    public static final double ALGAE_ON_TOP = 0.017;

    public static final double GEAR_RATIO = 100;
  }

  public final class GrabberConstants {
    public static final int JOINT_MOTOR_PORT = 16;
    public static final int GRAB_PORT = 17;
    public static final int LIMITSWITCH_PORT = 2;

    // public static final double TOP_HARD_LIMIT = 0.1;
    // public static final double BOTTOM_HARD_LIMIT = -0.44;
    // public static final double TOP_SOFT_LIMIT = 0.05;
    // public static final double BOTTOM_SOFT_LIMIT = -0.4 ;

    public static final double LEVEL_4 = 0.3949;
    public static final double GROUND_POSITION = 0.17181-5*0.006;
    public static final double FEEDER_POSITION = 0.6492056;
    public static final double BARGE_POSITION = 0.71209;
    public static final double REST_POSITION = 0.0;
    public static final double PROCESSOR_POSITION = 0.19181 + 12*0.006;
    public static final double ALGAE_ON_TOP = 0.2579;

    public static final double LEVEL_1_TO_3 = 0.17831+27*0.006;
    public static final double LEVEL_1 = 0.32824;


    public static final double GEAR_RATIO = 102.4;
    public static final double CURRENT_LIMIT = 12;
  }

  public final class SwerveConstants {
    public static final double STICK_ROTATION_DEADBAND = 0.05;
    public static final double STICK_DEADBAND = 0.05;
    public static final int X_AXIS_PORT = 0;
    public static final int Y_AXIS_PORT = 1;
    public static final int ROTATIONAL_AXIS_PORT = 2;

    public static final boolean FIELD_RELATIVE_MODE = true;

    //0
    public static final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 1;
      public static final int ANGLE_MOTOR_ID = 3;
      public static final int CANCODER_ID = 2;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-87.2);
    }
    //1
    public static final class Mod1 {
      public static final int DRIVE_MOTOR_ID = 4;
      public static final int ANGLE_MOTOR_ID = 6;
      public static final int CANCODER_ID = 5;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-61.25);
    }
    //2
    public static final class Mod3 {
      public static final int DRIVE_MOTOR_ID = 8;
      public static final int ANGLE_MOTOR_ID = 10;
      public static final int CANCODER_ID = 9;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-97+180);
    }
    //3
    public static final class Mod2 {
      public static final int DRIVE_MOTOR_ID = 11;
      public static final int ANGLE_MOTOR_ID = 13;
      public static final int CANCODER_ID = 12;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-162.24);
    }

    public static final double MAX_RADIANS_PER_SECOND = 12.773732;
    public static final int PIGEON_ID = 7;

    public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = COTSTalonFXSwerveConstants.SDS.MK4i
        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    public static final double TRACK_WIDTH = Units.inchesToMeters(30);
    public static final double WHEEL_BASE = Units.inchesToMeters(30);
    public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
    public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int ANGLE_CURRENT_LIMIT = 25;
    public static final int ANGLE_CURRENT_THRESHOLD = 40;
    public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int DRIVE_CURRENT_THRESHOLD = 60;
    public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Ramp Rates */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.25;

    /* PID Values */
    public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
    public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
    public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;

    public static final double DRIVE_KP = 0.46;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /* Drive Motor Characterization */
    public static final double DRIVE_KS = 1.5;
    public static final double DRIVE_KV = 1.51;
    public static final double DRIVE_KA = 0.21;

    /* Swerve Profiling Values */
    public static final double MAX_SPEED_METERS_PER_SECOND = 1.5;
    //was 4.5 changed to 1.5 for slow mode in demo
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10.0;

    /* Neutral Modes */
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
  }



}
