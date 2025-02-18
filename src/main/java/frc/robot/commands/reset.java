// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.stateReset;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class reset extends Command {
  /** Creates a new Reset. */
  public reset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevator);
  }
  boolean finish = false;
  boolean flag = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.m_elevator.getLimitSwitch()) {
      System.out.println("limit switch doesn't work");
      flag = true;
      RobotContainer.m_elevator.disable();
      RobotContainer.m_elevator.changeState(stateReset.NOT_INITIALIZED);
      finish = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_elevator.getLimitSwitch()) {
        System.out.println("limit switch works");
        RobotContainer.m_elevator.movePosition(RobotContainer.m_elevator.getPosition()+0.03);
    } else if( RobotContainer.m_elevator.getState() != stateReset.NOT_INITIALIZED){
      RobotContainer.m_elevator.changeState(stateReset.INITIALIZED);
      RobotContainer.m_elevator.setPosition(0);
      RobotContainer.m_elevator.movePosition(0);
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
