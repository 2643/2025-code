package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TeleopSwerve extends Command {

  private boolean initFlag = true;
  private static DoubleSupplier translationSup;
  private static DoubleSupplier strafeSup;
  private static DoubleSupplier rotationSup;
  private static BooleanSupplier robotCentricSup;
  GenericEntry rawrotationEntry = Shuffleboard.getTab("Swerve").add("Raw Rotation", 0).getEntry();
  GenericEntry rotationsupEntry =Shuffleboard.getTab("Swerve").add("Rotation Sup", 0).getEntry();
  public TeleopSwerve() {

    addRequirements(RobotContainer.s_Swerve);
  }

  private static double logAxis(double value) {
    return Math.copySign(Math.log(Math.abs(value) + 1) / Math.log(2), value);
  }

  private static double squareAxis(double value, double deadband) {
    value = MathUtil.applyDeadband(value, deadband);
    return Math.copySign(value * value, value);
  }

  @Override
  public void execute() {
    translationSup = () -> RobotContainer.driver.getRawAxis(1);
    strafeSup = () -> RobotContainer.driver.getRawAxis(0);
    rotationSup = () -> RobotContainer.driver.getRawAxis(4);
    robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean();

    if (initFlag) {
      RobotContainer.s_Swerve.resetModulesToAbsolute();
      RobotContainer.s_Swerve.zeroHeading();
      initFlag = false;
    }

    double rawRotation = rotationSup.getAsDouble();
    rawRotation = squareAxis(logAxis(rawRotation), Constants.stickRotationDeadband)
        * Constants.Swerve.maxAngularVelocity / 4;
    

    double translationVal = squareAxis(logAxis(translationSup.getAsDouble()), Constants.stickDeadband);
    double strafeVal = squareAxis(logAxis(strafeSup.getAsDouble()), Constants.stickDeadband);
    rawrotationEntry.setDouble(rawRotation);
    rotationsupEntry.setDouble(rotationSup.getAsDouble());
    SmartDashboard.putNumber("rawrotation", rawRotation);
    SmartDashboard.putNumber("rotationsup", rotationSup.getAsDouble());

    RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rawRotation,
        !robotCentricSup.getAsBoolean(),
        true);
        
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
