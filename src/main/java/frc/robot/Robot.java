// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ParallelCommands.ElevatorAndGrabberButtonStates;
import frc.robot.commands.ParallelCommands.ElevatorAndGrabberMovePos;
import frc.robot.commands.ParallelCommands.ResetAll;
import frc.robot.commands.Swerve.SwerveEncoderJoymode;
import frc.robot.commands.Swerve.SwerveTeleop;
import frc.robot.subsystems.Elevator.stateLevel;
import frc.robot.subsystems.Elevator.stateReset;
import frc.robot.subsystems.Grabber.GrabberPlacement;
import frc.robot.subsystems.Grabber.States;
;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  
  public GrabberPlacement curPlaceGrab;
  public stateLevel curPlaceElevator;

  public GenericEntry lowestVolts = Shuffleboard.getTab("teleop").add("Lowest Voltage", 0.0).getEntry();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    RobotContainer.s_Swerve.resetModulesToAbsolute();
    // CommandScheduler.getInstance().schedule(new SwerveAuto().withTimeout(2));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    //schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }


    
    //CommandScheduler.getInstance().setDefaultCommand(RobotContainer.s_Swerve, new SwerveTeleop());

    //CommandScheduler.getInstance().schedule(new SwerveAutoAlign());

    // curPlaceGrab = RobotContainer.s_Grabber.getPlacement();
    // curPlaceElevator = RobotContainer.s_Elevator.getLevel();
    // if(
    // RobotContainer.s_Grabber.getState() == States.INITIALIZING &&
    // RobotContainer.s_Elevator.getState() == stateReset.INITIALIZING){
    // CommandScheduler.getInstance().schedule(new ResetAll());
    // }

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    CameraServer.startAutomaticCapture();
  }
  

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    System.out.println("jegnergnergnkjergnkerngkerngnrgkjnekjgnerngenrgkreng");
    RobotContainer.s_Swerve.resetModulesToAbsolute();
    curPlaceGrab = RobotContainer.s_Grabber.getPlacement();
    curPlaceElevator = RobotContainer.s_Elevator.getLevel();

    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      if( RobotContainer.s_Grabber.getState() == States.INITIALIZING &&
    RobotContainer.s_Elevator.getState() == stateReset.INITIALIZING){
    CommandScheduler.getInstance().schedule(new ResetAll());
    }
    }
    CameraServer.startAutomaticCapture();

    
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("here");
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.s_Swerve, new SwerveTeleop());

    if (RobotContainer.operatorJoystick.getPOV() == 0) {
      CommandScheduler.getInstance().schedule(new ElevatorAndGrabberButtonStates(stateLevel.ALGAE_ON_TOP, GrabberPlacement.ALGAE_ON_TOP));
    }
    if (curPlaceGrab != RobotContainer.s_Grabber.getPlacement()
        && RobotContainer.s_Grabber.getState() == States.ENCODER) {
          System.out.println(RobotContainer.operatorJoystick.getPOV());
      if (curPlaceElevator != RobotContainer.s_Elevator.getLevel()
          && RobotContainer.s_Elevator.getState() == stateReset.INITIALIZED) {
            //System.out.println(RobotContainer.operatorJoystick.getPOV());
        
        curPlaceGrab = RobotContainer.s_Grabber.getPlacement();
        curPlaceElevator = RobotContainer.s_Elevator.getLevel();
        CommandScheduler.getInstance().schedule(new ElevatorAndGrabberMovePos(curPlaceGrab, curPlaceElevator));
      }
    }

    if (RobotContainer.driverJoystick.getPOV(0) == 0) {
      new SwerveEncoderJoymode(true).schedule();
    }
    if (RobotContainer.driverJoystick.getPOV(0) == 180) {
      new SwerveEncoderJoymode(false).schedule();
    }

    double battvolts = RobotController.getBatteryVoltage();
    double min = 100;
    if (min > battvolts) {
      min = battvolts;
      lowestVolts.setDouble(min); 
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
   
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.