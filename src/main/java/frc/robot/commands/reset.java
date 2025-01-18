// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.states;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class reset extends Command {
  boolean finish = false;
  /** Creates a new reset. */
  public reset() {
    addRequirements(RobotContainer.m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.m_Climb.getLimitSwitch()) {
      RobotContainer.m_Climb.current_state = states.NOT_INTIALIZED;
      finish = true;
    } else {
      RobotContainer.m_Climb.current_state = states.INTIALIZING;
    }
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_Climb.current_state == states.INTIALIZING && RobotContainer.m_Climb.getLimitSwitch()) {
      RobotContainer.m_Climb.move_motor(RobotContainer.m_Climb.get_pos()+0.3);
    } else {
      RobotContainer.m_Climb.current_state = states.INTIALIZED;
      RobotContainer.m_Climb.set_pos();
      RobotContainer.m_Climb.move_motor(0);
      finish = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
