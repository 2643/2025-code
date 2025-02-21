// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NextPos extends Command {
  /** Creates a new NextPos. */
  boolean finish;
  public NextPos() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Grabber);
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
    switch (RobotContainer.m_Grabber.getPlacement()){
      case REST:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.L1);
        System.out.println("L1");
        break;
      case L1:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.L2);
        System.out.println("L2");
        break;
      case L2:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.L3);
        System.out.println("L3");
        break;
      case L3:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.L4);
        System.out.println("L4");
        break;
      case L4:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.L1);
        System.out.println("L1");
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
