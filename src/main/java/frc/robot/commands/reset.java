// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.motor.stateReset;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class reset extends Command {
  /** Creates a new Reset. */
  public reset() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  boolean finish = false;
  boolean flag = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.m_motor.getLimitSwitch()) {
      flag = true;
      RobotContainer.m_motor.disable();
      RobotContainer.m_motor.changeState(stateReset.NOT_INITIALIZED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_motor.getLimitSwitch()) {
        RobotContainer.m_motor.movePosition(RobotContainer.m_motor.getPosition()+0.3);
    } else if (flag) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_motor.changeState(stateReset.INITIALIZED);
    RobotContainer.m_motor.setPosition(0);
    RobotContainer.m_motor.movePosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
