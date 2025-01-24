// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {


  public enum states {
    NOT_INTIALIZED,
    INTIALIZING,
    INTIALIZED
  }

  public states current_state = states.INTIALIZING;
  /** Creates a new Climb. */
  TalonFX motor = new TalonFX(13);
  TalonFXConfiguration config = new TalonFXConfiguration();
  public DigitalInput limitswitch1 = new DigitalInput(0);
  public CurrentLimitsConfigs currentconfig = new CurrentLimitsConfigs();

  public Climb() {



    var slot0Configs = config.Slot0; 
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    config.MotionMagic.MotionMagicAcceleration = 20;
    config.MotionMagic.MotionMagicCruiseVelocity = 20;
    currentconfig.StatorCurrentLimit = 3;

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(currentconfig);
    motor.setNeutralMode(NeutralModeValue.Brake);
    

  }

  public void move_motor(double pos) {
    motor.setControl(new MotionMagicVoltage(pos));
  }




  public void set_state(states target_state) {
    current_state = target_state;
  }

  public boolean getLimitSwitch(){
    return RobotContainer.m_Climb.limitswitch1.get();
  }

  public void disable_motor() {
    motor.disable();
  }

  public void set_pos() {
    motor.setPosition(0);
  }

  public double get_pos() {
    return motor.getPosition().getValueAsDouble();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (current_state) {
      case NOT_INTIALIZED:
        
        break;
    
      case INTIALIZING:
        
      case INTIALIZED:

    }
  }
}