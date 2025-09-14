package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.autoAim;

public class SwerveTeleop extends Command {

  private boolean initFlag = true;
  private static DoubleSupplier translationSup;
  private static DoubleSupplier strafeSup;
  private static DoubleSupplier rotationSup;
  static double rawRotation;
  static double rotationval;
  static double strafeVal;
  static double translationVal;
  static boolean fieldRelative;

  // static BooleanSupplier robotCentricSup;
  static double encoderkP = 0.38;
  static double encoderkI = 0.025;
  static double encoderkD = 0.002;
  static double ff = 0.09;

  Timer timer = new Timer();
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10000, 1000);
  ProfiledPIDController rotPid = new ProfiledPIDController(encoderkP, encoderkI, encoderkD, constraints);

  public static double save = 0;

  public SwerveTeleop() {
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
    double deadzone = 0.05;
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
    translationVal = RobotContainer.driverJoystick.getRawAxis(1);
    strafeVal = RobotContainer.driverJoystick.getRawAxis(0);


    translationSup = () -> RobotContainer.driverJoystick.getRawAxis(1);
    strafeSup = () -> RobotContainer.driverJoystick.getRawAxis(0);

    rotationSup = () -> RobotContainer.driverJoystick.getRawAxis(2);

    // robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean(); (work on
    // it if u need to)

    if (initFlag) {
      RobotContainer.s_Swerve.resetModulesToAbsolute();
      RobotContainer.s_Swerve.zeroHeading();
      initFlag = false;
    }

    rawRotation = rotationSup.getAsDouble();

    // translationVal = squareAxis(logAxis(translationSup.getAsDouble()),
    // Constants.stickDeadband + 0.3);

    if (RobotContainer.s_Vision.getAutoAim() != autoAim.NONE && RobotContainer.s_Vision.isApriltag() /*&& RobotContainer.s_Swerve.autoaimstate*/) {

      strafeVal = -RobotContainer.s_Vision.autostrafe()/4;
      rotationval = -RobotContainer.s_Vision.autoAngle()
          * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND /1.5/4;
      translationVal = -RobotContainer.s_Vision.autotrans()
          * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/4;
      fieldRelative = false;

    } else {
      translationVal = -squareAxis(logAxis(translationSup.getAsDouble()), Constants.SwerveConstants.STICK_DEADBAND) /2;
      strafeVal = -squareAxis(logAxis(strafeSup.getAsDouble()), Constants.SwerveConstants.STICK_DEADBAND) / 1.2 /2;
      rotationval = -squareAxis(logAxis(rawRotation), Constants.SwerveConstants.STICK_ROTATION_DEADBAND) * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND /2;
      fieldRelative = true;
    }

    /*
     * if (RobotContainer.encoderJoymodeState &&
     * RobotContainer.opboard.isConnected()
     * && !RobotContainer.autoAim.getAsBoolean()) {
     * rotationval =
     * rotPid.calculate(RobotContainer.s_Swerve.getGyroYaw().getDegrees(),
     * rawRotation) / 14
     * Constants.SwerveConstants.maxAngularVelocity / 3;
     * } else if (RobotContainer.encoderJoymodeState &&
     * RobotContainer.opboard.isConnected()
     * && RobotContainer.autoAim.getAsBoolean() &&
     * !RobotContainer.s_Vision.isApriltag()) {
     * rotationval =
     * rotPid.calculate(RobotContainer.s_Swerve.getGyroYaw().getDegrees(),
     * rawRotation) / 14
     * Constants.SwerveConstants.maxAngularVelocity / 3;
     * 
     * } else if (RobotContainer.opboard.isConnected() &&
     * RobotContainer.autoAim.getAsBoolean()
     * && RobotContainer.s_Vision.isApriltag()) {
     * rotationval = RobotContainer.s_Vision.autoAngle() *
     * Constants.SwerveConstants.maxAngularVelocity;
     * } else if (!RobotContainer.encoderJoymodeState) {
     * rotationval = squareAxis(logAxis(rawRotation),
     * Constants.stickRotationDeadband)
     * Constants.SwerveConstants.maxAngularVelocity / 4;
     * }
     */

    // rotPid.setP(RobotContainer.s_Swerve.pEntry.getDouble(encoderkP));
    // rotPid.setI(RobotContainer.s_Swerve.iEntry.getDouble(encoderkI));
    // rotPid.setD(RobotContainer.s_Swerve.dEntry.getDouble(encoderkD));

    // SmartDashboard.putNumber("rotationVal", rotationval);
    // RobotContainer.s_Swerve.targetrotValueEntry.setDouble(rotationval);

    RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND),
        rotationval, fieldRelative, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}