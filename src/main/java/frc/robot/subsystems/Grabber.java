// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */

  public enum States {
    NOT_INITIALIZED,
    INITIALIZING,
    INITIALIZED
  }

  SparkMax max = new SparkMax(4, MotorType.kBrushless);
  SparkMax maxLeader = new SparkMax(11, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
  SparkClosedLoopController maxController = max.getClosedLoopController();
  TalonFX turning = new TalonFX(0);
  TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  DigitalInput limitswitch = new DigitalInput(0);

  States init = States.NOT_INITIALIZED;
  
  public Grabber() {
    var slot0config = talonConfig.Slot0;
    var magicmotionconfig = talonConfig.MotionMagic;

    slot0config.kP = 0;
    slot0config.kI = 0;
    slot0config.kD = 0;

    magicmotionconfig.MotionMagicAcceleration = 0;
    magicmotionconfig.MotionMagicCruiseVelocity = 0;

    turning.getConfigurator().apply(talonConfig);

    config
        .follow(11,true)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        
    max.configure(config, null, null);

   
    maxLeader.set(.07);
  }

  public double getOutputCurrent(){
    return maxLeader.getOutputCurrent();
  }

  public States getState(){
    return init;
  }

  public void setPos(double pos){
    turning.setPosition(pos);
  }

  public void moveNeos(double speed){
    maxLeader.getClosedLoopController().setReference(speed, ControlType.kCurrent);
  }

  public void moveTurningMotor(double pos){
    turning.setControl(motion.withPosition(pos));
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

  public void setState(States state){
    init = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}