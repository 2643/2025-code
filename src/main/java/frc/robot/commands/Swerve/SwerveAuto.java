// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class SwerveAuto extends Command {
  /** Creates a new AutoSwerve. */
  public SwerveAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve);
  }
  double translationVal;
  double strafeVal;
  double rotationval;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("here");
    RobotContainer.s_Swerve.zeroHeading();
    RobotContainer.s_Swerve.resetModulesToAbsolute();
    // RobotContainer.s_Swerve.drive(new ChassisSpeeds());
    translationVal = 1;
    strafeVal = 0;
    rotationval = 0;
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND/2),
        rotationval, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}