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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.outtake;

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
  public enum IntakeOuttake {
    INTAKE,
    OUTTAKE
  }

  double target;

  SparkMax max = new SparkMax(3, MotorType.kBrushless);
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
  IntakeOuttake intakeOuttake = IntakeOuttake.INTAKE;

  GenericEntry targetPos = Shuffleboard.getTab("pos").add("target", 0).getEntry();

  static double auxFF = 0;
  static double grabberAngle;
  
  public Grabber() {
    var slot0config = talonConfig.Slot0;
    var magicmotionconfig = talonConfig.MotionMagic;
    turning.setInverted(true);

     

    slot0config.kP = 23;
    slot0config.kI = 0;
    slot0config.kD = 0;
    

    magicmotionconfig.MotionMagicAcceleration = 15;
    magicmotionconfig.MotionMagicCruiseVelocity = 15;

    turning.getConfigurator().apply(talonConfig);
    turning.setNeutralMode(NeutralModeValue.Brake);
    

    config
        .follow(11,true)
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(15);
    leaderConfig
        .idleMode(IdleMode.kBrake)
       .smartCurrentLimit(15);
        
    maxLeader.configure(leaderConfig, null, null);
    max.configure(config, null, null);


   
    // maxLeader.set(.07);
    SmartDashboard.putNumber("target", 0);
  }

  public double getOutputCurrent(){
    return maxLeader.getOutputCurrent();
  }

  public IntakeOuttake getIntakeOuttake(){
    return intakeOuttake;
  }

  public void setIntakeOutake(IntakeOuttake state){
    intakeOuttake = state;
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
  targetPos.setDouble(pos);
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
    return turning.getPosition().getValueAsDouble() / Constants.GRABBERGEARRATIO;
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
        System.out.println(getOutputCurrent());
        
      moveTurningMotor(targetPos.getDouble(0));
      // if (RobotContainer.m_ReefSwitch.getAsBoolean()){
      //   System.out.println(RobotContainer.m_Joystick.getRawAxis(2));
      //   moveL(RobotContainer.m_Joystick.getRawAxis(2));
      // } else {
      //   moveFG(RobotContainer.m_FeederGround.getAsBoolean());
      // }
        if (getOutputCurrent() >= Constants.CURRENTLIMIT){
            //maxLeader.stopMotor();
            switch (intakeOuttake) {
              case INTAKE:
                maxLeader.set(-0.3);
                break;
            
              case OUTTAKE:
                //maxLeader.set(0.2);
                break;
            }
          

        }
      break;
    }
  }
}