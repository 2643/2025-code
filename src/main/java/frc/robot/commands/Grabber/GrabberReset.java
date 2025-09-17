package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.States;

public class GrabberReset extends Command {
  private boolean finish = false;

  /** Creates a new GrabberReset. */
  public GrabberReset() {
    addRequirements(RobotContainer.s_Grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.s_Grabber.getLimitSwitch()) {
      RobotContainer.s_Grabber.setState(States.NOT_INITIALIZED);
      finish = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Grabber.getLimitSwitch()) {
      RobotContainer.s_Grabber.moveTurningMotor(RobotContainer.s_Grabber.getPos() + 0.03);
      
    } 
    if (!RobotContainer.s_Grabber.getLimitSwitch() && RobotContainer.s_Grabber.getState() != States.NOT_INITIALIZED) {
      RobotContainer.s_Grabber.setState(States.INITIALIZED);
      finish = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Grabber.setPos(0);
    RobotContainer.s_Grabber.moveTurningMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
