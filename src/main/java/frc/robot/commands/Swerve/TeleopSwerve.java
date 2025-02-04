package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private static DoubleSupplier rotationSupX;
  private static DoubleSupplier rotationSupY;
  //private static DoubleSupplier rotationSup;
  private static BooleanSupplier robotCentricSup;
  GenericEntry rawrotationEntry = Shuffleboard.getTab("Swerve").add("Raw Rotation", 0).getEntry();
  GenericEntry rotationsupEntry =Shuffleboard.getTab("Swerve").add("Rotation Sup", 0).getEntry();

  static double encoderkP = 0.08;
  static double encoderkI = 0.00;
  static double encoderkD = 0.005;
  static double ff = 0.005;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10000, 10000);
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
    double deadzone = 0.2;
        if ((x <= deadzone && x >= -deadzone) && (y <= deadzone && y >= -deadzone)) return save;
        
        double angle = Math.toDegrees(Math.atan2(-y, x));
        
        if (angle < 0){
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
    //rotationSup = () -> RobotContainer.driver.getRawAxis(2);
    rotationSupX = () -> RobotContainer.driver.getRawAxis(2);
    rotationSupY = () -> RobotContainer.driver.getRawAxis(5);
    //robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean(); (work on it if u need to)

    if (initFlag) {
      RobotContainer.s_Swerve.resetModulesToAbsolute();
      // RobotContainer.s_Swerve.zeroHeading();
      initFlag = false;
    }
    
    double rawRotation = getJoystickAngle(rotationSupX.getAsDouble(), rotationSupY.getAsDouble());
    // rawRotation = squareAxis(logAxis(rawRotation), Constants.stickRotation\Deadband)
    //     * Constants.Swerve.maxAngularVelocity / 4;
    

    double translationVal = squareAxis(logAxis(translationSup.getAsDouble()), Constants.stickDeadband+0.2);
    double strafeVal = squareAxis(logAxis(strafeSup.getAsDouble()), Constants.stickDeadband+0.2);

    double rotationVal = rotPid.calculate(RobotContainer.s_Swerve.getGyroYaw().getDegrees(), rawRotation) / 14* Constants.Swerve.maxAngularVelocity/3;
    
    rawrotationEntry.setDouble(rawRotation);
    rawrotationEntry.setDouble(rotationVal);
    
    
    SmartDashboard.putNumber("rotationVal", rotationVal);
    SmartDashboard.putNumber("rawrotation", rawRotation);
    SmartDashboard.putNumber("rotationsupX", rotationSupX.getAsDouble());
    SmartDashboard.putNumber("rotationsupY", rotationSupY.getAsDouble());

    RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal,
        true,
        true);
        
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}