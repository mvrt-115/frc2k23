// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignAndExtend;
import frc.robot.commands.AutonRunner;
import frc.robot.commands.AutonScoreTwoAndLevel;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Localization;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeHPStation;
import frc.robot.commands.Leveling;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.Score;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Intake2.INTAKE_TYPE;
import frc.robot.utils.JoystickIO;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  // private final Localization localization; //Utils camera

  private Elevator elevator;
  private Intake2 intake = new Intake2(INTAKE_TYPE.wheeled);

  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private final JoystickIO driveJoystick = new JoystickIO(Constants.SwerveDrivetrain.kDriveJoystickPort, true, false);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);
  private final SendableChooser<Command> autonSelector = new SendableChooser<>();

  private final Trigger levelTrigger;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.JoystickControls.invertJoystickX)
      driveJoystick.invertJoystickX();
    if (Constants.JoystickControls.invertJoystickY)
      driveJoystick.invertJoystickY();
    if (Constants.JoystickControls.invertJoystickW)
      driveJoystick.invertJoystickW();
    driveJoystick.invertLeftStick();
    //elevator = new Elevator();

  //  localization = new Localization(swerveDrivetrain);
    driveJoystick.button(0);
    elevator = new Elevator(new TalonFX(Constants.Elevator.MOTOR_ID), new TalonFX(Constants.Elevator.MOTOR_ID2));
    swerveDrivetrain.setDefaultCommand(new SwerveJoystickCommand(
      swerveDrivetrain, 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveXAxis), 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveYAxis), 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveWAxis), 
      driveJoystick.button(Constants.SwerveDrivetrain.kDriveFieldOrientButtonIdx),
      driveJoystick, elevator));
      elevator.setDefaultCommand(new ManualElevator(elevator, () -> -operatorJoystick.getRawAxis(1)*0.6));
      
    // Configure the trigger bindings
    configureBindings();
    //elevator = new Elevator();
    //elevator.setDefaultCommand(new SetElevatorHeight(elevator));

    levelTrigger = driveJoystick.button(2);
    levelTrigger.onTrue( new DriveForward(swerveDrivetrain, 1, 0.5).andThen(new Leveling(swerveDrivetrain)) ).onFalse( new DriveForward(swerveDrivetrain, 0, 0));
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
    driveJoystick.button(4).onTrue(new InstantCommand(() -> swerveDrivetrain.resetOdometry(new Pose2d(0,0,new Rotation2d())))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)));

    autonSelector.addOption("ExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ExitLevel"));
    autonSelector.addOption("ScoreExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ScoreExitLevel"));
    autonSelector.addOption("ScoreLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ScoreLevel"));
    autonSelector.setDefaultOption("ScoreTwiceLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ScoreTwiceLevel"));
    SmartDashboard.putData("Auton Selector", autonSelector);
  
    //Align to nearest column on click
    // Pose2d nearestCol = localization.getClosestScoringLoc();
    // driveJoystick.button(4).whileTrue(new Align(swerveDrivetrain, localization, nearestCol)).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));
    
    //Brake baby brake
    driveJoystick.button(5).onTrue(new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)));

    //No braking
    driveJoystick.button(6).onTrue(new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Coast)));
   // operatorJoystick.b().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT)).onFalse(new ManualElevator(elevator, 0));
   // operatorJoystick.a().onTrue(new SetElevatorHeight(elevator, Csonstants.Elevator.CONE_MID_HEIGHT)).onFalse(new ManualElevator(elevator, 0));
  //  driverController.x().whileTrue(intake.runIn()).onFalse(intake.stop());
  //  driverController.y().whileTrue(intake.runOut()).onFalse(intake.stop());
    operatorJoystick.x().onTrue(new IntakeHPStation(elevator, intake)).onFalse(new SetElevatorHeight(elevator, 400).alongWith(intake.stop()));
 //   operatorJoystick.b().onTrue(intake.runOut().andThen(new WaitCommand(1).andThen(new SetElevatorHeight(elevator, 100).alongWith(intake.stop()))));
    operatorJoystick.b().onTrue(new SetElevatorHeight(elevator, 400).alongWith(intake.stop()));
    operatorJoystick.y().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT+550)).onFalse(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT-3700).alongWith(intake.runOut()));//.alongWith(intake.runOut()).andThen(new SetElevatorHeight(elevator, 100)));//.andThen(new SetElevatorHeight(elevator, 100).alongWith(intake.stop())));
    operatorJoystick.a().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT)).onFalse(intake.runOut());//.onFalse(intake.runOut().andThen(new WaitCommand(1).andThen(new SetElevatorHeight(elevator, 100).alongWith(intake.stop()))));
    operatorJoystick.button(6).onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CUBE_MID_HEIGHT+550)).onFalse(intake.runOut());
    operatorJoystick.button(5).onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CUBE_HIGH_HEIGHT+550)).onFalse(intake.runOut());
    // operatorJoystick.b().onTrue(new ManualElevator(elevator, 0.2).alongWith(intake.runIn())).onFalse(new ManualElevator(elevator, -0.2).alongWith(intake.runOut()).andThen(new ManualElevator(elevator, 0)));
 //   operatorJoystick.a().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT));
 //   operatorJoystick.b().onTrue(new SetElevatorHeight(elevator, 0));
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