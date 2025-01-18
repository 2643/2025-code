package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TeleopSwerve extends Command {

  private boolean initFlag = true;
  // private double encoderOffset;
  // private double betterEncoderAngle = 0;

  private static DoubleSupplier translationSup;
  private static DoubleSupplier strafeSup;
  private static DoubleSupplier rotationSup;
  private static BooleanSupplier robotCentricSup;

  // private static final double encoderkP = 0.08;
  // private static final double encoderkI = 0.00;
  // private static final double encoderkD = 0.005;
  // private static final double ff = 0.005;

  //private final ProfiledPIDController rotPid;

  // private static final GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional", encoderkP).getEntry();
  // private static final GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral", encoderkI).getEntry();
  // private static final GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative", encoderkD).getEntry();
  // private static final GenericEntry ffEntry = Shuffleboard.getTab("PID").add("ff", ff).getEntry();
  // private static final GenericEntry rotValEntry = Shuffleboard.getTab("PID").add("outVal", 0).getEntry();
  // private static final GenericEntry betterEncoderEntry = Shuffleboard.getTab("Swerve").add("Encoder Target Angle", 0).getEntry();

  public TeleopSwerve() {
    addRequirements(RobotContainer.s_Swerve);
    // rotPid = new ProfiledPIDController(encoderkP, encoderkI, encoderkD, new TrapezoidProfile.Constraints(10000, 10000));
    // rotPid.enableContinuousInput(0, 360);
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
    // rotPid.setP(pEntry.getDouble(encoderkP));
    // rotPid.setI(iEntry.getDouble(encoderkI));
    // rotPid.setD(dEntry.getDouble(encoderkD));
    // betterEncoderEntry.setDouble(betterEncoderAngle);

    translationSup = () -> -RobotContainer.driver.getRawAxis(1);
    strafeSup = () -> -RobotContainer.driver.getRawAxis(2);
    rotationSup = () -> RobotContainer.driver.getRawAxis(4);
    robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean();

    if (initFlag) {
      RobotContainer.s_Swerve.zeroHeading();
      // encoderOffset = RobotContainer.driver.getRawAxis(3);
      initFlag = false;
    }

    double rawRotation = rotationSup.getAsDouble();
    rawRotation =  squareAxis(logAxis(rawRotation), Constants.stickRotationDeadband) * Constants.Swerve.maxAngularVelocity / 4;

    

    // //convert to degrees
    // if (rawRotation < 0) {
    //   betterEncoderAngle = (2 + rawRotation) * 180;
    // } else if (rawRotation > 2) {
    //   betterEncoderAngle = (rawRotation - 2) * 180;
    // } else {
    //   betterEncoderAngle = rawRotation * 180;
    // }
    // betterEncoderAngle -= 180;

    double translationVal = squareAxis(logAxis(translationSup.getAsDouble()), Constants.stickDeadband);
    double strafeVal = squareAxis(logAxis(strafeSup.getAsDouble()), Constants.stickDeadband);

    // rotValEntry.setDouble(rawRotation);

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
