// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {


  public enum states {
    NOT_INTIALIZED,
    INTIALIZING,
    INTIALIZED,
    BUTTON_CLICKED_ACTIVATE,
    CLIMB_ENDED,
    WENT_BACK_UP,
    DONE_BACK_UP
  }


  public states current_state = states.INTIALIZING;
  /** Creates a new ClimbConstants. */
  TalonFX motor = new TalonFX(Constants.ClimbConstants.MOTOR_PORT);
  TalonFXConfiguration config = new TalonFXConfiguration();
  public DigitalInput limitswitch1 = new DigitalInput(Constants.ClimbConstants.LIMITSWITCH_PORT);

  GenericEntry climbLimitswitchEntry = Shuffleboard.getTab("Climb").add("Climb limitswitch", limitswitch1.get()).getEntry();
  GenericEntry climbCurrentPosEntry = Shuffleboard.getTab("Climb").add("Climb Current Positon", 0).getEntry();
  GenericEntry climbCurrentStateEntry = Shuffleboard.getTab("Climb").add("Climb Current State", current_state.toString()).getEntry();
  GenericEntry climbTargetPosEntry = Shuffleboard.getTab("Climb").add("Climb Target Position", 0).getEntry();
  GenericEntry climbOutputCurrent = Shuffleboard.getTab("Climb").add("Climb Output Current", 0).getEntry();

  double targetPos = 0;
  public Climb() {



    var slot0Configs = config.Slot0; 
    slot0Configs.kP = 25; 
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    //slot0Configs.kG = -1.152;

    config.MotionMagic.MotionMagicAcceleration = 15;
    config.MotionMagic.MotionMagicCruiseVelocity = 15;
    config.CurrentLimits.SupplyCurrentLimit = 17; //THIS IS WAY TOO LOW, CHANGE THIS TO BE BETTER!!
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    

  }
//hi
  public void move_motor(double pos) {
    if (current_state == states.INTIALIZED) {
      if (get_pos() < Constants.ClimbConstants.BOTTOM_HARD_LIMIT|| get_pos() > Constants.ClimbConstants.TOP_HARD_LIMIT) {
        motor.stopMotor();
      }
      else if (get_pos() < Constants.ClimbConstants.BOTTOM_SOFT_LIMIT) {
        targetPos = Constants.ClimbConstants.BOTTOM_SOFT_LIMIT - 10;
      }
      else if (get_pos() > Constants.ClimbConstants.TOP_SOFT_LIMIT) {
       targetPos = Constants.ClimbConstants.TOP_SOFT_LIMIT +10;
      }
      
    }
    //System.out.println(String.valueOf(pos));
    motor.setControl(new MotionMagicVoltage(pos*Constants.ClimbConstants.GEAR_RATIO));
  }

  public void set_state(states target_state) {
    current_state = target_state;
  }

  public boolean getLimitSwitch(){
    return limitswitch1.get();
  }

  public void disable_motor() {
    motor.disable();
  }

  public void set_pos() {
    motor.setPosition(0);
  }
  public states getState() {
    return current_state;
  }

  public double get_pos() {
    return motor.getPosition().getValueAsDouble()/Constants.ClimbConstants.GEAR_RATIO;
  }

  public double get_current() {
    return motor.getStatorCurrent().getValueAsDouble();
  }




  @Override
  public void periodic() {
    climbLimitswitchEntry.setBoolean(getLimitSwitch());
    climbCurrentPosEntry.setDouble(get_pos());
    climbCurrentStateEntry.setString(current_state.toString());
    climbTargetPosEntry.setDouble(targetPos);
    climbOutputCurrent.setDouble(get_current());
    // This method will be called once per scheduler run
    switch (current_state) {
      case NOT_INTIALIZED:
        break;
      case INTIALIZING:
        break;
      case INTIALIZED:
        break;
      case BUTTON_CLICKED_ACTIVATE:
        move_motor(Constants.ClimbConstants.MOVE_TO_CAGE_POSITION);
        targetPos = Constants.ClimbConstants.MOVE_TO_CAGE_POSITION;
        if(get_current() >= Constants.ClimbConstants.CURRENT_LIMIT){
          motor.stopMotor();
        }
        current_state = states.CLIMB_ENDED;
      case CLIMB_ENDED:
        break;
      case WENT_BACK_UP:
        move_motor(20/Constants.ClimbConstants.GEAR_RATIO);
        targetPos = 20/Constants.ClimbConstants.GEAR_RATIO;
        current_state = states.DONE_BACK_UP;
      case DONE_BACK_UP:
        break;
      default: 
        break;

    }
  }
}