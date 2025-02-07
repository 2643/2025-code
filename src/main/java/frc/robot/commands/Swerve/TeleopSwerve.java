package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TeleopSwerve extends Command {

  private boolean initFlag = true;
  private static DoubleSupplier translationSup;
  private static DoubleSupplier strafeSup;
  private static DoubleSupplier rotationSupX;
  private static DoubleSupplier rotationSupY;
  private static DoubleSupplier rotationSup;
  static double rawRotation;
  static double rotationval;

  // static BooleanSupplier robotCentricSup;
  static double encoderkP = 0.38;
  static double encoderkI = 0.025;
  static double encoderkD = 0.002;
  static double ff = 0.09;

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10000, 1000);
  ProfiledPIDController rotPid = new ProfiledPIDController(encoderkP, encoderkI, encoderkD, constraints);

  public static double save = 0;

  public TeleopSwerve() {
    addRequirements(RobotContainer.s_Swerve);
    rotPid.enableContinuousInput(0, 360);
  }

  private static double logAxis(double value) {
    return Math.copySign(Math.log(Math.abs(value) + 1) / Math.log(2), value);
  }

  private static double squareAxis(double value, double deadband) {
    value = MathUtil.applyDeadband(value, deadband);
    return Math.copySign(value * value, value);
  }

  public static double getJoystickAngle(double x, double y) {
    double deadzone = 0.4;
    if ((x <= deadzone && x >= -deadzone) && (y <= deadzone && y >= -deadzone))
      return save;

    double angle = Math.toDegrees(Math.atan2(-y, x));

    if (angle < 0) {
      angle += 360;
    }
    save = angle;
    return save;

  }


  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    translationSup = () -> RobotContainer.driver.getRawAxis(1);
    strafeSup = () -> RobotContainer.driver.getRawAxis(0);
    if (!RobotContainer.encoderJoymode.getAsBoolean()) {
      rotationSupX = () -> RobotContainer.driver.getRawAxis(2);
      rotationSupY = () -> RobotContainer.driver.getRawAxis(5);
    } else {
      rotationSup = () -> RobotContainer.driver.getRawAxis(2);
    }
    // robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean(); (work on
    // it if u need to)

    if (initFlag) {
      RobotContainer.s_Swerve.resetModulesToAbsolute();
      // RobotContainer.s_Swerve.zeroHeading();
      initFlag = false;
    }
    if (!RobotContainer.encoderJoymode.getAsBoolean()) {
      rawRotation = getJoystickAngle(rotationSupX.getAsDouble(), rotationSupY.getAsDouble());
    } else {
      rawRotation = rotationSup.getAsDouble();
    }

    double translationVal = squareAxis(logAxis(translationSup.getAsDouble()), Constants.stickDeadband + 0.3);
    double strafeVal = squareAxis(logAxis(strafeSup.getAsDouble()), Constants.stickDeadband + 0.3);

    if (!RobotContainer.encoderJoymode.getAsBoolean()) {
      rotationval = rotPid.calculate(RobotContainer.s_Swerve.getGyroYaw().getDegrees(), rawRotation) / 14
          * Constants.Swerve.maxAngularVelocity / 4;
    } else if (RobotContainer.encoderJoymode.getAsBoolean()) {
      rotationval = squareAxis(logAxis(rawRotation), Constants.stickRotationDeadband)
          * Constants.Swerve.maxAngularVelocity / 4;
    }

    /*
     * if (RobotContainer.encoderJoymode.getAsBoolean() &&
     * RobotContainer.opboard.isConnected()
     * && !RobotContainer.autoAim.getAsBoolean()) {
     * rotationval =
     * rotPid.calculate(RobotContainer.s_Swerve.getGyroYaw().getDegrees(),
     * rawRotation) / 14
     * Constants.Swerve.maxAngularVelocity / 3;
     * } else if (RobotContainer.encoderJoymode.getAsBoolean() &&
     * RobotContainer.opboard.isConnected()
     * && RobotContainer.autoAim.getAsBoolean() &&
     * !RobotContainer.s_Vision.isApriltag()) {
     * rotationval =
     * rotPid.calculate(RobotContainer.s_Swerve.getGyroYaw().getDegrees(),
     * rawRotation) / 14
     * Constants.Swerve.maxAngularVelocity / 3;
     * 
     * } else if (RobotContainer.opboard.isConnected() &&
     * RobotContainer.autoAim.getAsBoolean()
     * && RobotContainer.s_Vision.isApriltag()) {
     * rotationval = RobotContainer.s_Vision.autoAngle() *
     * Constants.Swerve.maxAngularVelocity;
     * } else if (!RobotContainer.encoderJoymode.getAsBoolean()) {
     * rotationval = squareAxis(logAxis(rawRotation),
     * Constants.stickRotationDeadband)
     * Constants.Swerve.maxAngularVelocity / 4;
     * }
     */

    rotPid.setP(RobotContainer.s_Swerve.pEntry.getDouble(encoderkP));
    rotPid.setI(RobotContainer.s_Swerve.iEntry.getDouble(encoderkI));
    rotPid.setD(RobotContainer.s_Swerve.dEntry.getDouble(encoderkD));

    SmartDashboard.putNumber("rotationVal", rotationval);
    RobotContainer.s_Swerve.targetrotValueEntry.setDouble(rotationval);

    RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationval,
        true,
        true);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}