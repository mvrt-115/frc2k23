// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Align;
import frc.robot.commands.AutonPathExample;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Localization;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Leveling;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.JoystickIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Localization localization; //Utils camera

  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private final JoystickIO driveJoystick = new JoystickIO(Constants.SwerveDrivetrain.kDriveJoystickPort, true, false);
  private final SendableChooser<Command> autonSelector = new SendableChooser<>();

  private final Trigger levelTrigger;

  // Replace with CommandPS4Controller or CommandJoystick if needed
 /* private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);

      private final JoystickButton intakeButton = new JoystickButton(m_driverController, 1);
      private final JoystickButton outtakeButton = new JoystickButton(m_driverController, 2);
      private final JoystickButton manualButton = new JoystickButton(m_driverController, 3);*/
  
  private final Joystick driverController = new Joystick(OperatorConstants.kDriverControllerPort);

  private final JoystickButton intakeButton = new JoystickButton(driverController, 1);
  private final JoystickButton outtakeButton = new JoystickButton(driverController, 2);

  private final JoystickButton manualIntake = new JoystickButton(driverController, 3);
  private final JoystickButton manualOuttake = new JoystickButton(driverController, 4);

  private Intake intake;
  private Elevator elevator;
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

    localization = new Localization(swerveDrivetrain);
    driveJoystick.button(0);
    swerveDrivetrain.setDefaultCommand(new SwerveJoystickCommand(
      swerveDrivetrain, 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveXAxis), 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveYAxis), 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveWAxis), 
      driveJoystick.button(Constants.SwerveDrivetrain.kDriveFieldOrientButtonIdx),
      driveJoystick));
      
    // Configure the trigger bindings
    intake = new Intake(Intake.INTAKE_TYPE.claw); /// SPECIFY WHETHER WHEELED OR CLAW INTAKE
    configureBindings();
    elevator = new Elevator();
    //elevator.setDefaultCommand(new SetElevatorHeight(elevator));

    levelTrigger = driveJoystick.button(2);
    levelTrigger.onTrue( new DriveForward(swerveDrivetrain, 1, 0.5).andThen(new Leveling(swerveDrivetrain)) );
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
    driveJoystick.button(2).onTrue(new InstantCommand(() -> swerveDrivetrain.resetOdometry(new Pose2d(0,0,new Rotation2d())))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)));

    autonSelector.setDefaultOption("Example", new AutonPathExample(swerveDrivetrain));
    SmartDashboard.putData("Auton Selector", autonSelector);
    //Align to nearest column on click
    Pose2d nearestCol = Constants.VisionConstants.kRedScoreCols.get(5);//localization.getClosestScoringLoc();
    driveJoystick.button(4).whileTrue(new Align(swerveDrivetrain, localization)).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));
    driveJoystick.button(5).onTrue(new SequentialCommandGroup(
      new Align(swerveDrivetrain, localization),
      new SetElevatorHeight(elevator, 0),
      intake.intakeElement()
    ));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  //  outtakeButton.whileTrue(intake.outtakeElement()); //Run outtake when A is pressed
   // intakeButton.whileTrue(intake.intakeElement()); //Intend to intake when B is pressed

  // manualButton.whileTrue(intake.manualIntake()); //Run manual intake when X is pressed

    intakeButton.onTrue(intake.intakeElement());
    outtakeButton.onFalse(intake.outtakeElement());

    manualIntake.whileTrue(intake.manualIntake()).onFalse(intake.manualOuttake()); //Run manual intake when X is pressed
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