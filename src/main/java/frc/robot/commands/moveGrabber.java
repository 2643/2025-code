// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveGrabber extends Command {
  GrabberPlacement place;
  boolean finish;
  /** Creates a new moveGrabber. */
  public moveGrabber(GrabberPlacement place) {
    addRequirements(RobotContainer.m_Grabber);
    this.place = place;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (place == GrabberPlacement.L1L2L3){
      RobotContainer.m_Grabber.moveTurningMotor(Constants.L1L2L3);
    } else if (place == GrabberPlacement.L4){
      RobotContainer.m_Grabber.moveTurningMotor(Constants.L4);
    } else if (place == GrabberPlacement.GROUND){
      RobotContainer.m_Grabber.moveTurningMotor(Constants.GROUND);
    } else if (place == GrabberPlacement.FEEDER){
      RobotContainer.m_Grabber.moveTurningMotor(Constants.FEEDER);
    } else if (place == GrabberPlacement.BARGE){
      RobotContainer.m_Grabber.moveTurningMotor(Constants.BARGE);
    } else if (place == GrabberPlacement.REST){
      RobotContainer.m_Grabber.moveTurningMotor(Constants.REST);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
