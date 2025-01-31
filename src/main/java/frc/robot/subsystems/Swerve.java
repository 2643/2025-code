package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModules;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  public SwerveDriveOdometry swerveOdometry;
  public SwerveModules[] mSwerveMods;
  public static Pigeon2 gyro;
  public ChassisSpeeds mSpeeds;
  public SwerveDrivePoseEstimator robotPose;

  Field2d field = new Field2d();
  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field").add("2025-Field", field)
      .withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));
  GenericEntry swerveAngleEntry = Shuffleboard.getTab("Swerve").add("Swerve Angle", 0).getEntry();
  GenericEntry encoderAngleEntry = Shuffleboard.getTab("Swerve").add("Encoder Angle", 0).getEntry();

  
  public Swerve() {

    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);

    // Initialize the Swerve modules array
    mSwerveMods = new SwerveModules[] {
        new SwerveModules(0, Constants.Swerve.Mod0.driveMotorID, Constants.Swerve.Mod0.angleMotorID,
            Constants.Swerve.Mod0.canCoderID, Constants.Swerve.Mod0.angleOffset),
        new SwerveModules(1, Constants.Swerve.Mod1.driveMotorID, Constants.Swerve.Mod1.angleMotorID,
            Constants.Swerve.Mod1.canCoderID, Constants.Swerve.Mod1.angleOffset),
        new SwerveModules(2, Constants.Swerve.Mod2.driveMotorID, Constants.Swerve.Mod2.angleMotorID,
            Constants.Swerve.Mod2.canCoderID, Constants.Swerve.Mod2.angleOffset),
        new SwerveModules(3, Constants.Swerve.Mod3.driveMotorID, Constants.Swerve.Mod3.angleMotorID,
            Constants.Swerve.Mod3.canCoderID, Constants.Swerve.Mod3.angleOffset)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    robotPose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(),
        getPose());

  }

  // return modulestates of all swerve modules in a list
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModules mod : mSwerveMods) {
      states[mod.modNumber] = mod.getState();
    }
    return states;
  }

  // return modulestates of all swerve modules in a list
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModules mod : mSwerveMods) {
      positions[mod.modNumber] = mod.getPosition();
    }
    return positions;
  }

  // move to set location
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));
    mSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModules mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.modNumber], isOpenLoop);
    
    }
  }

  // set speeds of all modules and move to current location
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    mSpeeds = speeds;

    for (SwerveModules mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.modNumber], true);
    }
  }

  
  public ChassisSpeeds getChassisSpeeds() {
    return mSpeeds;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // renormalize wheelspeeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    // move to desired state with optimization
    for (SwerveModules mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.modNumber], true);
    }
  }

  // return current position of swerveodometry (the chassis internal positioning
  // system)
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  // Change the position of the swerveodometry
  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  // get the direction of robot face
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  // change the direct of the robot face (use the current position to get there)
  public void setHeading(Rotation2d heading) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  // reset heading to 0 degrees
  public void zeroHeading() {
     swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
         new Pose2d(getPose().getTranslation(), new Rotation2d()));
  //   for (SwerveModules mod : mSwerveMods) {
  //     mod.setHeadingZero();
  // }
  gyro.setYaw(0);
    
}

  // returns curent rotation
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  // reset all modules to absolute postion (what was recorded previously, accounts
  // for offsets)
  public void resetModulesToAbsolute() {
    for (SwerveModules mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getGyroYaw(), getModulePositions());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());

    for (SwerveModules mod : mSwerveMods) {
      SmartDashboard.putNumber("Module " + mod.modNumber + " Desired Angle: ", mod.Desired());
      SmartDashboard.putNumber("Mod " + mod.modNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.modNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.modNumber + " Velocity", mod.getState().speedMetersPerSecond);

    }
  }
}