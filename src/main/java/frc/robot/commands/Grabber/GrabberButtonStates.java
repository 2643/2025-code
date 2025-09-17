// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberButtonStates extends Command {
  /** Creates a new MovePos. */
  GrabberPlacement place;
  boolean finish;
  public GrabberButtonStates(GrabberPlacement place) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Grabber);
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
      case HIGHALGAE -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.HIGHALGAE);
      case LOWALGAE -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.LOWALGAE);
      case PROCESSOR -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.PROCESSOR);
      case BARGE -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.BARGE);
      case FEEDER -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.FEEDER);
      case GROUND -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.GROUND);
      case ALGAE_ON_TOP -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.ALGAE_ON_TOP);
      default -> {
          }
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}