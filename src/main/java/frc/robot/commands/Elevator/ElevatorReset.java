// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateReset;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorReset extends Command {
  /** Creates a new Reset. */
  public ElevatorReset() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Elevator);
  }
  public boolean finish = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.s_Elevator.getLimitSwitch()) {
      //RobotContainer.s_Elevator.disable();
      RobotContainer.s_Elevator.changeState(stateReset.NOT_INITIALIZED);
      // finish=true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(finish);
    if (RobotContainer.s_Elevator.getLimitSwitch()) {
        RobotContainer.s_Elevator.movePosition(RobotContainer.s_Elevator.getPosition()-0.03);
        RobotContainer.s_Elevator.changeState(stateReset.INITIALIZED);
    } 
    if(!RobotContainer.s_Elevator.getLimitSwitch() && RobotContainer.s_Elevator.getState() != stateReset.NOT_INITIALIZED){
      finish = true;
    }
    if(RobotContainer.s_Elevator.getState() == stateReset.NOT_INITIALIZED && !RobotContainer.s_Elevator.getLimitSwitch() ){
      RobotContainer.s_Elevator.movePosition(RobotContainer.s_Elevator.getPosition()+0.03);
     
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (RobotContainer.s_Elevator.getState() == stateReset.INITIALIZED){
    //   RobotContainer.s_Elevator.movePosition(-2 / Constants.ElevatorConstants.GEAR_RATIO);
    // }
    RobotContainer.s_Elevator.setPosition(0);
    RobotContainer.s_Elevator.movePosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
