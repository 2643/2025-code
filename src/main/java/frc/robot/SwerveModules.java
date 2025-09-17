package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class SwerveModules extends SubsystemBase {
  /** Creates a new Swerve. */
  private TalonFX drivemotor;
  private TalonFX turnmotor;
  private Rotation2d angleOfset;
  private CANcoder cancoder;
  private DutyCycleOut dco = new DutyCycleOut(0);
  private VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.DRIVE_KS, Constants.SwerveConstants.DRIVE_KV, Constants.SwerveConstants.DRIVE_KA);
  private PositionVoltage pv = new PositionVoltage(0);
  public int modNumber;
  private double desiredstatereturn;
  private double simDistance;
  private Rotation2d simAngle;

  // constructor intialization
  public SwerveModules(int modNumber, int drivemotor, int turnmotor, int cancoder, Rotation2d angleOfset) {
    this.drivemotor = new TalonFX(drivemotor);
    this.turnmotor = new TalonFX(turnmotor);
    this.cancoder = new CANcoder(cancoder);
    this.angleOfset = angleOfset;
    this.modNumber = modNumber;
    this.simAngle = new Rotation2d(0);
    this.simDistance = 0.0;
    
    // this.turnmotor.setInverted(inversion);
    this.drivemotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    this.drivemotor.getConfigurator().setPosition(0);

    this.turnmotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    this.turnmotor.getConfigurator().setPosition(0);

    this.cancoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    //this.cancoder.getConfigurator().setPosition(0);
    resetToAbsolute();
  }


  // returns rotation2d object of the absolute cancoder rotation, of specified
  // module
  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
  }

  public void setHeadingZero() {
    cancoder.getConfigurator().setPosition(0);
    turnmotor.getConfigurator().setPosition(0);
    //this.drivemotor.getConfigurator().setPosition(0);
  }

  // removes offset from absolute position (for calibration)
  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations()-angleOfset.getRotations();
    turnmotor.getConfigurator().setPosition(absolutePosition);

    
    
  }

  // returns SwerveModulePosition object of the drivemotor position in distance
  // and Rotation2d with rotation of turnmotor
  public SwerveModulePosition getPosition() {
    if (Robot.isSimulation()) {
      return new SwerveModulePosition(simDistance, simAngle);
  } else {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(drivemotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.WHEEL_CIRCUMFERENCE),
        Rotation2d.fromRotations(turnmotor.getPosition().getValueAsDouble()));
  }

  }

  // returns SwerveModuleState object with the speed of the wheel of the module
  // derived from the drivemotor, and angle of module derived from the turnmotor
  // position
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.rotationsToMeters(drivemotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.WHEEL_CIRCUMFERENCE),
        Rotation2d.fromRotations(turnmotor.getPosition().getValueAsDouble()));

  }

  public void setSimPosition(double distance, Rotation2d angle) {
    // Store the distance and angle in your module state/position
    this.simDistance = distance;
    this.simAngle = angle;
}

  // optimize wheel rotation path
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState.optimize(getState().angle);
    desiredstatereturn = desiredState.angle.getRotations();
    turnmotor.setControl(pv.withPosition(desiredState.angle.getRotations()));
    setSpeed(desiredState, isOpenLoop);
  }

  public double Desired(){
    return desiredstatereturn;
  }

  

  // Based on if the parameter is open or closed loop the velocity and/or
  // feedforward is set
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      dco.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
      drivemotor.setControl(dco);
    } else {
      driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
          Constants.SwerveConstants.WHEEL_CIRCUMFERENCE);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      drivemotor.setControl(driveVelocity);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}