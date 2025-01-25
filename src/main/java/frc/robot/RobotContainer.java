// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.intake;
import frc.robot.commands.moveL1;
import frc.robot.commands.moveL2;
import frc.robot.commands.moveL3;
import frc.robot.commands.moveL4;
import frc.robot.commands.outtake;
import frc.robot.commands.reset;
import frc.robot.commands.stopNeos;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final Grabber m_Grabber = new Grabber();
  public static final Joystick m_Joystick = new Joystick(0);
  public static final JoystickButton m_IntakeButton = new JoystickButton(m_Joystick, 1);
  public static final JoystickButton m_OuttakeButton = new JoystickButton(m_Joystick, 2);
  public static final JoystickButton m_L1 = new JoystickButton(m_Joystick, 3);
  public static final JoystickButton m_L2 = new JoystickButton(m_Joystick, 4);
  public static final JoystickButton m_L3 = new JoystickButton(m_Joystick, 5);
  public static final JoystickButton m_L4 = new JoystickButton(m_Joystick, 6);
  public static final JoystickButton m_Reset = new JoystickButton(m_Joystick, 7);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_IntakeButton.onTrue(new intake());
    m_OuttakeButton.onTrue(new outtake());
    m_IntakeButton.onFalse(new stopNeos());
    m_OuttakeButton.onFalse(new stopNeos());
    m_L1.onTrue(new moveL1());
    m_L2.onTrue(new moveL2());
    m_L3.onTrue(new moveL3());
    m_L4.onTrue(new moveL4());
    m_Reset.onTrue(new reset());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
