// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateLevel;
import frc.robot.subsystems.Vision.autoAim;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */

  public enum States {
    INITIALIZED,
    INITIALIZING,
    NOT_INITIALIZED,
    ENCODER
  }

  public enum GrabberPlacement {
    L1,
    L2,
    L3,
    L4,
    GROUND,
    FEEDER,
    REST,
    BARGE,
    PROCESSOR,
    HIGHALGAE,
    LOWALGAE,
    ALGAE_ON_TOP,
    AUTO_LEVEL_4
  }

  public enum IntakeOuttake {
    INTAKE,
    OUTTAKE,
    NOTHING
  }

  static double target;
  static IntakeOuttake intakeOuttake = IntakeOuttake.NOTHING;

  static Timer timer = new Timer();
  TalonFX turning = new TalonFX(Constants.GrabberConstants.JOINT_MOTOR_PORT);
  DigitalInput limitswitch = new DigitalInput(Constants.GrabberConstants.LIMITSWITCH_PORT);

  TalonFXConfiguration talonGrabConfig = new TalonFXConfiguration();
  TalonFXConfiguration talonWristConfig = new TalonFXConfiguration();
  MotionMagicVoltage motion = new MotionMagicVoltage(0);

  States curStates = States.INITIALIZING;
  GrabberPlacement curPlacement = GrabberPlacement.REST;
  TalonFX grab = new TalonFX(Constants.GrabberConstants.GRAB_PORT);

  static double auxFF = 0;
  static double grabberAngle;

  GenericEntry jointLimitswitchEntry = Shuffleboard.getTab("Grabber").add("Joint limitswitch", limitswitch.get())
      .getEntry();
  GenericEntry jontCurrentPosEntry = Shuffleboard.getTab("Grabber").add("Joint Current Positon", 0).getEntry();
  GenericEntry jointTargetPosEntry = Shuffleboard.getTab("Grabber").add("JointTarget Position", target).getEntry();
  GenericEntry jointInitStateEntry = Shuffleboard.getTab("Grabber").add("Joint State Init", curStates.toString())
      .getEntry();
  GenericEntry jointLevelStateEntry = Shuffleboard.getTab("Grabber").add("Joint State Level", curPlacement.toString())
      .getEntry();
  GenericEntry grabberIntakeOuttakeEntry = Shuffleboard.getTab("Grabber")
      .add("Grabber Intake Outtake", intakeOuttake.toString()).getEntry();

  public Grabber() {
    var slot0config = talonWristConfig.Slot0;
    var magicmotionconfig = talonWristConfig.MotionMagic;
    // turning.setInverted(true);

    slot0config.kP = 13;
    slot0config.kI = 0;
    slot0config.kD = 0;

    magicmotionconfig.MotionMagicAcceleration = 50;
    magicmotionconfig.MotionMagicCruiseVelocity = 50;

    talonWristConfig.CurrentLimits.StatorCurrentLimit = 35;
    talonWristConfig.CurrentLimits.StatorCurrentLimitEnable = true;;

    turning.getConfigurator().apply(talonWristConfig);

    grab.setNeutralMode(NeutralModeValue.Brake);

    talonGrabConfig.CurrentLimits.StatorCurrentLimit = 60;
    talonGrabConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    grab.getConfigurator().apply(talonGrabConfig);

    SmartDashboard.putNumber("target", 0);
  }

  public States getState() {
    return curStates;
  }

  public void setPos(double pos) {
    turning.setPosition(pos);
  }

  public void moveTurningMotor(double pos) {
    target = pos;
    System.out.println(pos);
    // if (curStates == States.ENCODER) {

    //   if (getPos() > Constants.GrabberConstants.BOTTOM_HARD_LIMIT
    //       || getPos() < Constants.GrabberConstants.TOP_HARD_LIMIT) {
    //     turning.disable();
    //     target = 0;
    //   }
    //   if (getPos() > Constants.GrabberConstants.BOTTOM_SOFT_LIMIT) {
    //     target = Constants.GrabberConstants.REST_POSITION;
    //   }
    //   if (getPos() < Constants.GrabberConstants.TOP_SOFT_LIMIT) {
    //     target = Constants.GrabberConstants.BARGE_POSITION;
    //   }
    // }

    turning.setControl(motion.withPosition(target * Constants.GrabberConstants.GEAR_RATIO));
  }

  public boolean getLimitSwitch() {
    return limitswitch.get();
  }

  public double getPos() {
    return turning.getPosition().getValueAsDouble() / Constants.GrabberConstants.GEAR_RATIO;
  }

  public GrabberPlacement getPlacement() {
    return curPlacement;
  }

  public void setState(States state) {
    curStates = state;
  }

  public void setPlacement(GrabberPlacement place) {
    curPlacement = place;
  }

  public double getOutputCurrent() {
    return turning.getStatorCurrent().getValueAsDouble();
  }

  public void setIntakeOutake(IntakeOuttake state) {
    intakeOuttake = state;
  }

  public static void Ticker(double time, boolean outake) {
    timer.start();
    if (timer.hasElapsed(time)) {
      timer.stop();
      timer.reset();
      if (outake) {
        intakeOuttake = IntakeOuttake.NOTHING;
      }
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    grabberAngle = 0 + ((getPos() * 360 / Constants.GrabberConstants.GEAR_RATIO)); // try removing gear ratios
    auxFF = /* FFWEntry.getDouble(-0.15) */ 0.13 * Math.sin(Math.toRadians(grabberAngle)); // -0.128
    // auxFF = 0.35 * Math.sin(Math.toRadians((getPos()-25)));
    jointLimitswitchEntry.setBoolean(getLimitSwitch());
    jontCurrentPosEntry.setDouble(getPos());
    jointTargetPosEntry.setDouble(target);
    jointInitStateEntry.setString(curStates.toString());
    jointLevelStateEntry.setString(curPlacement.toString());
    grabberIntakeOuttakeEntry.setString(intakeOuttake.toString());
    SmartDashboard.putNumber("Output", getOutputCurrent());

    switch (curStates) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        break;
      case INITIALIZED:
        setState(States.ENCODER);
        break;
      case ENCODER:
        switch (intakeOuttake) {
          
          case INTAKE:
            if(RobotContainer.s_Vision.getAutoAim() == autoAim.MIDDLE){
              grab.set(-0.8);
            }
            else{
              grab.set(-0.6);
            }
            
            break;
          case OUTTAKE:
            if(getPlacement() == GrabberPlacement.BARGE){
              grab.set(0.8);
            }
            else{
              grab.set(0.3);
            }
           
            Ticker(1, true);
            break;
          case NOTHING:
            grab.set(0);
            break;
        }
        break;
    }
  }
}
