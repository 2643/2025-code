// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.states;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbReset extends Command {
  boolean finish = false;


  /** Creates a new reset. */
  public ClimbReset() {
    addRequirements(RobotContainer.s_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.s_Climb.getLimitSwitch()) {
      RobotContainer.s_Climb.set_state(states.NOT_INTIALIZED);
      RobotContainer.s_Climb.disable_motor();
      System.out.println(RobotContainer.s_Climb.getState());
      finish = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Climb.current_state == states.INTIALIZING && RobotContainer.s_Climb.getLimitSwitch()) {
      System.out.println("here");
      RobotContainer.s_Climb.move_motor(RobotContainer.s_Climb.get_pos() - 0.3);
    } else if (RobotContainer.s_Climb.current_state == states.INTIALIZING && !RobotContainer.s_Climb.getLimitSwitch()) {
      System.out.println("done intializing");
      RobotContainer.s_Climb.set_state(states.INTIALIZED);
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Climb.set_pos();
    RobotContainer.s_Climb.move_motor(0);
    if(RobotContainer.s_Climb.current_state == states.INTIALIZED){
      RobotContainer.s_Climb.move_motor(20/Constants.ClimbConstants.GEAR_RATIO);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
