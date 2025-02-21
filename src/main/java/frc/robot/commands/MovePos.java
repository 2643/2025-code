// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovePos extends Command {
  /** Creates a new MovePos. */
  GrabberPlacement place;
  boolean finish;
  public MovePos(GrabberPlacement place) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Grabber);
    this.place = place;
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
    switch (place){
      case HIGHALGAE:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.HIGHALGAE);
        System.out.println("high algae");
        break;
      case LOWALGAE:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.LOWALGAE);
        System.out.println("low algae");
        break;
      case PROCESSOR:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.PROCESSOR);
        System.out.println("processor");
        break;
      case RAMP:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.RAMP);
        System.out.println("ramp");
        break;
      case BARGE:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.BARGE);
        System.out.println("barge");
        break;
      case FEEDER:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.FEEDER);
        System.out.println("feeder");
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
