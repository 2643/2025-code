// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.states;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class reset extends Command {

  /** Creates a new reset. */
  public reset() {
    addRequirements(RobotContainer.m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Climb.set_state(states.NOT_INTIALIZED);
  }
  boolean getLimitSwitch(){
    return RobotContainer.m_Climb.limitswitch1.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getLimitSwitch() == true) {
      RobotContainer.m_Climb.reset_pos();
      RobotContainer.m_Climb.set_state(states.INTIALIZING);
    } else {
      RobotContainer.m_Climb.set_state(states.INTIALIZED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
