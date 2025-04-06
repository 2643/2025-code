// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb.states;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbMove extends Command {
  /** Creates a new button. */
  public ClimbMove() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Climb);
  }


  boolean finish = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("here");
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(RobotContainer.s_Climb.getState() == states.CLIMB_ENDED) {
      RobotContainer.s_Climb.set_state(states.WENT_BACK_UP);
    } else if (RobotContainer.s_Climb.getState() == states.DONE_BACK_UP || RobotContainer.s_Climb.getState() == states.INTIALIZED) {
      RobotContainer.s_Climb.set_state(states.BUTTON_CLICKED_ACTIVATE);
    }
  }
    
//
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
