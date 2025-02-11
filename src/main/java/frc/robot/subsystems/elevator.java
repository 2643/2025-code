// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder;
import frc.robot.commands.ground;
import frc.robot.commands.moveElevator;
import frc.robot.commands.reset;


public class elevator extends SubsystemBase {
  TalonFX motor = new TalonFX(13);
  static DigitalInput limitSwitch = new DigitalInput(0);
  GenericEntry limitswitchEntry = Shuffleboard.getTab("elevator").add("limitswitch",false).getEntry();
  GenericEntry currentPosEntry = Shuffleboard.getTab("elevator").add("Current Positon", 0).getEntry();

  public static enum stateLevel {
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

  static stateReset currentState = stateReset.INITIALIZING;
  static stateLevel currentLevel = stateLevel.L1;
  
  static double currentPos;

  public elevator() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    
    // Set the position to 0 rotations for initial use
    motor.setPosition(0);
    
    // Get Position and Velocity
    var position = motor.getPosition();
    var velocity = motor.getVelocity();
    
    //set motor acceleration for motion magic
    configs.MotionMagic.MotionMagicAcceleration = 5;
    configs.MotionMagic.MotionMagicCruiseVelocity = 5;

    // Write these configs to the TalonFX
    motor.getConfigurator().apply(configs);
  }

  public void movePosition(double pos) {
    currentPos = pos;
    if (getPosition() <= Constants.BottomHard || getPosition() >= Constants.TopHard) {
      motor.disable();
    } else if (getPosition() < Constants.BottomSoft) {
      System.out.println("hi");
      currentPos = Constants.BottomSoft+10;
    } else if (getPosition() > Constants.TopSoft) {
      System.out.println("hi");
      currentPos = Constants.TopSoft-10;    
    } 
    motor.setControl(new MotionMagicVoltage(currentPos));
  }

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public stateLevel moveWithEncoder(double axisVal) {
    // System.out.println("encoder");
    if (axisVal <= -0.75 && axisVal >= -1) {
      return stateLevel.L1;
    } else if (axisVal <= 0 && axisVal >= -0.75) {
      return stateLevel.L2;
    } else if (axisVal <= 0.75 && axisVal >= 1) {
      return stateLevel.L3;
    } else {
      return stateLevel.L4;
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

  public static stateLevel getLevel() {
    return currentLevel;
  }

  private stateLevel isCurrentPos = stateLevel.L1;

  @Override
  public void periodic() {
    limitswitchEntry.setBoolean(getLimitSwitch());
    SmartDashboard.putBoolean("Limitswitch", getLimitSwitch());
    SmartDashboard.putString("Level", getLevel().toString());
    SmartDashboard.putString("State", getState().toString()); 
    SmartDashboard.putNumber("Target Position", currentPos);
    SmartDashboard.putNumber("Current Position", getPosition());

    switch (currentState) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        break;
      case INITIALIZED:
        if (RobotContainer.encoderGroundSwitch.getAsBoolean()) {
          currentLevel = moveWithEncoder(RobotContainer.driver.getRawAxis(2));
          if (isCurrentPos != getLevel()) {
            isCurrentPos = getLevel();
            CommandScheduler.getInstance().schedule(new moveElevator(isCurrentPos));
          }
        } else {
          if (RobotContainer.feederGroundSwitch.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(new feeder());
          } else {
            CommandScheduler.getInstance().schedule(new ground());
          }
        }
      default:
    }
    // This method will be called once per scheduler run
    //moveWithEncoder(RobotContainer.driver.getRawAxis(0));
  }
}
