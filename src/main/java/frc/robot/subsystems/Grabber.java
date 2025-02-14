// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */

  public enum States {
    INITIALIZED,
    INITIALIZING,
    NOT_INITIALIZED,
    ENCODER
  }
  public enum GrabberPlacement {
    L1L2L3,
    L4,
    GROUND,
    FEEDER,
    REST,
    BARGE,
    PROCESSOR
  }

  SparkMax max = new SparkMax(4, MotorType.kBrushless);
  SparkMax maxLeader = new SparkMax(11, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkMaxConfig leaderConfig = new SparkMaxConfig();
  MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
  SparkClosedLoopController maxController = max.getClosedLoopController();
  TalonFX turning = new TalonFX(13);
  TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  DigitalInput limitswitch = new DigitalInput(0);

  States curStates = States.INITIALIZING;
  GrabberPlacement curPlacement = GrabberPlacement.REST;

  static double auxFF = 0;
  static double grabberAngle;
  
  public Grabber() {
    var slot0config = talonConfig.Slot0;
    var magicmotionconfig = talonConfig.MotionMagic;

     

    slot0config.kP = 3;
    slot0config.kI = 0;
    slot0config.kD = 0;

    magicmotionconfig.MotionMagicAcceleration = 2;
    magicmotionconfig.MotionMagicCruiseVelocity = 2;

    turning.getConfigurator().apply(talonConfig);
    turning.setNeutralMode(NeutralModeValue.Coast);

    config
        .follow(11,true)
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(1);
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(1);
        
    maxLeader.configure(leaderConfig, null, null);
    max.configure(config, null, null);


   
    // maxLeader.set(.07);
  }

  public double getOutputCurrent(){
    return maxLeader.getOutputCurrent();
  }

  public States getState(){
    return curStates;
  }

  public void setPos(double pos){
    turning.setPosition(pos);
  }

  public void moveNeos(double speed){ 
    maxLeader.set(speed);
  }

  public void moveTurningMotor(double pos){
    
    if (curStates == States.ENCODER){
      System.out.println("here");
    
      if (getPos() < Constants.BottomHard || getPos() > Constants.TopHard) {
        turning.disable();
        pos = 0;
      }
      else if (getPos() < Constants.BottomSoft) {
        pos = Constants.BottomHard + 1;
      }
      else if (getPos() > Constants.TopSoft) {
      pos = Constants.TopSoft - 1;
      }
  }
  turning.setControl(motion.withPosition(pos * Constants.GRABBERGEARRATIO));
}
  public boolean getLimitSwitch(){
    return limitswitch.get();
  }
  public void disableMotor(){
    turning.disable();
    maxLeader.disable();
    max.disable();
  }
  public double getPos(){
    return turning.getPosition().getValueAsDouble();
  }
  public GrabberPlacement getPlacement(){
    return curPlacement;
  }

  public void setState(States state){
    curStates = state;
  }
  public void setPlacement(GrabberPlacement place){
    curPlacement = place;
  }
  public void moveL(double axis){
    if (curStates != States.ENCODER){
      System.out.println("kys");
      return;
    }
    if (axis <= -0.95 && axis >= -0.98) {
      setPlacement(GrabberPlacement.GROUND);
    } else if (axis <= -0.06 && axis >= -0.1) {
      setPlacement(GrabberPlacement.L1L2L3);
    } else if (axis <= -0.53 && axis >= -0.56) {
      setPlacement(GrabberPlacement.REST);
    } else if (axis <= -0.27 && axis >= -0.29){
      setPlacement(GrabberPlacement.L4);
    }
  }
  public void moveFG(boolean feeder){
    if(curStates != States.ENCODER){
      return;
    }
    if (feeder){
      //move feeder
      setPlacement(GrabberPlacement.FEEDER);
    } else {
      //move floor
      setPlacement(GrabberPlacement.GROUND);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // grabberAngle = 0 + ((getPos()*360/Constants.GRABBERGEARRATIO) - (RobotContainer.m_Grabber.getPos()*360/Constants.GRABBERGEARRATIO)); //try removing gear ratios
    // auxFF = /*FFWEntry.getDouble(-0.15)*/ -0.35 * Math.sin(Math.toRadians(grabberAngle)); //-0.128 
    auxFF = 0.35 * Math.sin(Math.toRadians((getPos()-25)));
    turning.setControl(motion.withFeedForward(auxFF));
    SmartDashboard.putNumber("no", auxFF);
    SmartDashboard.putNumber("eruthr", getPos());
    SmartDashboard.putNumber("angel", Math.sin(Math.toRadians((getPos()))));
    SmartDashboard.putBoolean("limit", getLimitSwitch());
    switch (curStates) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        break;
      case INITIALIZED:
        setState(States.ENCODER); 
      case ENCODER:
      System.out.println("ENCODER");
      // if (RobotContainer.m_ReefSwitch.getAsBoolean()){
      //   System.out.println(RobotContainer.m_Joystick.getRawAxis(2));
      //   moveL(RobotContainer.m_Joystick.getRawAxis(2));
      // } else {
      //   moveFG(RobotContainer.m_FeederGround.getAsBoolean());
      // }
      break;
    }
  }
}