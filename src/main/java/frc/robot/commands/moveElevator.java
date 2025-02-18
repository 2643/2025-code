// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.elevator.stateLevel;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveElevator extends Command {

  stateLevel state;
  /** Creates a new moveElevator. */
  public moveElevator(stateLevel state) {
    addRequirements(RobotContainer.m_elevator);
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (state == stateLevel.L1) { 
      RobotContainer.m_elevator.movePosition(Constants.L2);
    } else if (state == stateLevel.L2) {
      RobotContainer.m_elevator.movePosition(Constants.L2);
    } else if (state == stateLevel.L3) {
      RobotContainer.m_elevator.movePosition(Constants.L3);
    } else if (state == stateLevel.L4) {
      RobotContainer.m_elevator.movePosition(Constants.L4);
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
