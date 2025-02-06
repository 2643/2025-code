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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.reset;

public class Climb extends SubsystemBase {


  public enum states {
    NOT_INTIALIZED,
    INTIALIZING,
    INTIALIZED,
    BUTTON_CLICKED_ACTIVATE,
    CLIMB_ENDED
  }


  public states current_state = states.INTIALIZING;
  /** Creates a new Climb. */
  TalonFX motor = new TalonFX(13);
  TalonFXConfiguration config = new TalonFXConfiguration();
  public DigitalInput limitswitch1 = new DigitalInput(0);
  double targetPos = 0;
  public Climb() {



    var slot0Configs = config.Slot0; 
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    //slot0Configs.kG = -1.152;

    config.MotionMagic.MotionMagicAcceleration = 2;
    config.MotionMagic.MotionMagicCruiseVelocity = 2;
    config.CurrentLimits.SupplyCurrentLimit = 2;
    

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    

  }
//hi
  public void move_motor(double pos) {
    if (current_state == states.INTIALIZED) {
      if (get_pos() < Constants.BottomHard || get_pos() > Constants.TopHard) {
        motor.stopMotor();
      }
      else if (get_pos() < Constants.BottomSoft) {
        targetPos = Constants.BottomHard - 1000;
      }
      else if (get_pos() > Constants.TopSoft) {
       targetPos = Constants.TopHard +1000;
      }
      
    }
    
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

  public double get_current() {
    return motor.getStatorCurrent().getValueAsDouble();
  }




  @Override
  public void periodic() {
    SmartDashboard.putNumber("currentpos", get_pos());
    SmartDashboard.putBoolean("huai te shih", getLimitSwitch());
    SmartDashboard.putNumber("outputcurrent", get_current());
    SmartDashboard.putString("state", current_state.toString());
    SmartDashboard.putNumber("target Pos", targetPos);
    // This method will be called once per scheduler run
    switch (current_state) {
      case NOT_INTIALIZED:
        break;
      case INTIALIZING:
        CommandScheduler.getInstance().schedule(new reset());
      case INTIALIZED:
        break;
      case BUTTON_CLICKED_ACTIVATE:
        move_motor(Constants.MoveToCage);
        targetPos = Constants.MoveToCage;
        if(get_current() >= Constants.CageCurrent){
          motor.stopMotor();
        }
        current_state = states.CLIMB_ENDED;
      case CLIMB_ENDED:
        break;
      default: 


    }
  }
}