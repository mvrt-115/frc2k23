// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.Intake2.INTAKE_TYPE;
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
  private Elevator elevator;
  private Intake2 intake = new Intake2(INTAKE_TYPE.wheeled);

  // Replace with CommandPS4Controller or CommandJoystick if needed
 /* private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);

      private final JoystickButton intakeButton = new JoystickButton(m_driverController, 1);
      private final JoystickButton outtakeButton = new JoystickButton(m_driverController, 2);
      private final JoystickButton manualButton = new JoystickButton(m_driverController, 3);*/
  
      private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // private final JoystickButton intakeButton = new JoystickButton(driverController, 1);
  // private final JoystickButton outtakeButton = new JoystickButton(driverController, 2);

  // private final JoystickButton manualIntake = new JoystickButton(driverController, 3);
  // private final JoystickButton manualOuttake = new JoystickButton(driverController, 4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    elevator = new Elevator(new TalonFX(Constants.Elevator.MOTOR_ID), new TalonFX(Constants.Elevator.MOTOR_ID2));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // elevator.setDefaultCommand(new SetElevatorHeight(elevator, 0.0));
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    driverController.x().whileTrue(new ManualElevator(elevator, 0.2)).onFalse(new ManualElevator(elevator, 0));
    driverController.b().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT)).onFalse(new ManualElevator(elevator, 0));
    driverController.a().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT)).onFalse(new ManualElevator(elevator, 0));
  //  driverController.x().whileTrue(intake.runIn()).onFalse(intake.stop());
  //  driverController.y().whileTrue(intake.runOut()).onFalse(intake.stop());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

//    driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
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