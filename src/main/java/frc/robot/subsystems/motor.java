// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class motor extends SubsystemBase {
  static TalonFX motor = new TalonFX(0);
  static DigitalInput limitSwitch = new DigitalInput(0);
  /** Creates a new motor. */
  enum stateLevel {
    L1,
    L2,
    L3,
    L4,
    Reset
  }
  public enum stateReset {
    NOT_INITIALIZED,
    INITIALIZING,
    INITIALIZED

  }

  public stateReset currentState = stateReset.INITIALIZING;
public motor() {
  TalonFXConfiguration configs = new TalonFXConfiguration();
  // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
  configs.Slot0.kP = 1;
  configs.Slot0.kI = 0;
  configs.Slot0.kD = 10;
 
  // Write these configs to the TalonFX
  motor.getConfigurator().apply(configs);
 
  // Set the position to 0 rotations for initial use
  motor.setPosition(0);
 
  // Get Position and Velocity
  var position = motor.getPosition();
  var velocity = motor.getVelocity();
 
 //set motor acceleration for motion magic
  configs.MotionMagic.MotionMagicAcceleration = 200;
  configs.MotionMagic.MotionMagicCruiseVelocity = 200;

  }

  public static void movePosition(double position) {
    motor.setControl(new MotionMagicVoltage(position));
  }

  public static void setPosition(double position) {
    motor.setPosition(position);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public void moveWithEncoder(double axisVal) {
    if (axisVal <= -0.75 && axisVal >= -1) {
      movePosition(200);
    } else if (axisVal <= 0 && axisVal >= -0.75) {
      movePosition(200);
    } else if (axisVal <= 0.75 && axisVal >= 1) {
      movePosition(200);
    } else {
      movePosition(0);
    }
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void disable() {
    motor.disable();
  }

  public void changeState(stateReset state) {
    currentState = state;
  }

  public stateReset getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveWithEncoder(RobotContainer.driver.getRawAxis(0));
  }
}
