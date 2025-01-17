// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {


  public enum states {
    NOT_INTIALIZED,
    INTIALIZING,
    INTIALIZED
  }

  public states current_state = states.NOT_INTIALIZED;
  /** Creates a new Climb. */
  TalonFX motor = new TalonFX(0);
  TalonFXConfiguration config = new TalonFXConfiguration();
  public DigitalInput limitswitch1 = new DigitalInput(0);

  public Climb() {



    var slot0Configs = config.Slot0; 
    slot0Configs.kP = 1;
    slot0Configs.kI = 9;
    slot0Configs.kD = 7;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    

  }

  public void move_motor(double pos) {
    motor.setControl(new MotionMagicVoltage(pos));
  }

  public void reset_pos() {
    motor.setControl(new DutyCycleOut(1));
  }

  public void reset() {
    motor.setPosition(0);
  }

  public void set_state(states target_state) {
    current_state = target_state;
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}