// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Align;
import frc.robot.commands.AutonPathExample;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.JoystickIO;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Localization localization; //Utils camera

  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private final JoystickIO driveJoystick = new JoystickIO(Constants.SwerveDrivetrain.kDriveJoystickPort, true, false);
  private final SendableChooser<Command> autonSelector = new SendableChooser<>();

  //private final Trigger levelTrigger;

  private Elevator elevator;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //elevator = new Elevator();

    localization = new Localization(swerveDrivetrain);
      driveJoystick.button(0);
    swerveDrivetrain.setDefaultCommand(new SwerveJoystickCommand(
      swerveDrivetrain, 
      () -> ((Constants.JoystickControls.invertJoystickX) ?-1:1)* driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveXAxis), 
      () -> ((Constants.JoystickControls.invertJoystickY) ?-1:1)* driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveYAxis), 
      () -> ((Constants.JoystickControls.invertJoystickW) ?-1:1) *  driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveWAxis), 
      driveJoystick.button(Constants.SwerveDrivetrain.kDriveFieldOrientButtonIdx),
      driveJoystick, elevator));
      
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    driveJoystick.button(3).onTrue(new InstantCommand(() -> swerveDrivetrain.resetModules()));
    driveJoystick.button(2).onTrue(new InstantCommand(() -> swerveDrivetrain.zeroHeading()));

    autonSelector.setDefaultOption("Example", new AutonPathExample(swerveDrivetrain));
    SmartDashboard.putData("Auton Selector", autonSelector);
  
    //Align to nearest column on click
    Pose2d nearestCol = localization.getClosestScoringLoc();
    driveJoystick.button(4).whileTrue(new Align(swerveDrivetrain, localization, nearestCol)).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonSelector.getSelected();
  }

  public void putTestCommand() {
    swerveDrivetrain.setupTests();
  }
}