// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Swerve.AutoSwerve;
import frc.robot.commands.Swerve.TeleopSwerve;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(AutoSwerve subsystem) {
    return Commands.sequence(new TeleopSwerve());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
