// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ParallelCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Elevator.ElevatorBumperDown;
import frc.robot.commands.Grabber.GrabberBumperDown;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndGrabberBumperDown extends ParallelCommandGroup {
  /** Creates a new ElevatorAndGrabberBumperDown. */
  public ElevatorAndGrabberBumperDown() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ElevatorBumperDown(), new GrabberBumperDown());
  }
}
