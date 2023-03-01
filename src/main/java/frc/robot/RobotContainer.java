// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
import frc.robot.subsystems.Intake2.INTAKE_TYPE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private CANdleLEDSystem leds = new CANdleLEDSystem();

  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private final JoystickIO driveJoystick = new JoystickIO(Constants.SwerveDrivetrain.kDriveJoystickPort, true, false);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);
  private final SendableChooser<Command> autonSelector = new SendableChooser<>();

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
    elevator = new Elevator();
    swerveDrivetrain.setDefaultCommand(new SwerveJoystickCommand(
      swerveDrivetrain, 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveXAxis), 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveYAxis), 
      () -> driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveWAxis), 
      driveJoystick.button(Constants.SwerveDrivetrain.kDriveFieldOrientButtonIdx),
      driveJoystick, elevator)
    );
    
    // ELEVATOR MANUAL
    elevator.setDefaultCommand(
      new ManualElevator(elevator, () -> -operatorJoystick.getRawAxis(1)*0.1)
    ); // used to 0.4, makes slower speed
      
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
    
    driveJoystick.button(3).onTrue(new InstantCommand(() -> swerveDrivetrain.resetModules()));
    driveJoystick.button(4).onTrue(new InstantCommand(() -> swerveDrivetrain.resetOdometry(new Pose2d(0,0,new Rotation2d())))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)));

    autonSelector.addOption("ExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ExitLevel"));
    autonSelector.addOption("ExitLevel2", new AutonRunner(swerveDrivetrain, elevator, intake, "ExitLevel2"));
    // autonSelector.addOption("ScoreExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ScoreExitLevel"));
    autonSelector.addOption("DONOTHING", new PrintCommand("hi"));
    autonSelector.addOption("OnlyLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ScoreLevel"));
    // autonSelector.setDefaultOption("ScoreTwiceLevel", new AutonRunner(swerveDrivetrain, elevator, intake, "ScoreTwiceLevel"));
    SmartDashboard.putData("Auton Selector", autonSelector);
  
    //Align to nearest column on click
    // Pose2d nearestCol = localization.getClosestScoringLoc();
    // driveJoystick.button(4).whileTrue(new Align(swerveDrivetrain, localization, nearestCol)).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));

    // AUTO LEVEL
    driveJoystick.button(2).onTrue(
      new SequentialCommandGroup(
        new DriveForward(swerveDrivetrain, Constants.Leveling.driveForwardMPS, Constants.Leveling.driveForwardTime),
        new Leveling(swerveDrivetrain) 
      )
    ).onFalse( 
      new InstantCommand(() -> swerveDrivetrain.stopModules())
    );
    
    //Brake baby brake
    driveJoystick.button(5).onTrue(new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)));

    //No braking
    driveJoystick.button(6).onTrue(new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Coast)));

    // HP INTAKE
    operatorJoystick.x().onTrue(
      new IntakeHPStation(elevator, intake)
    ).onFalse(
      new SetElevatorHeight(elevator, 1500).alongWith(intake.stop())
    );

    // RETURN TO NEUTRAL
    operatorJoystick.b().onTrue(
      new ParallelCommandGroup(
        new SetElevatorHeight(elevator, 1500),
        intake.stop()
      )
    ).onFalse(
      new InstantCommand(() -> elevator.runMotor(0))
    );

    // SCORE CONE MID 
    operatorJoystick.y().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT+550)
    ).onFalse(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT-3100).alongWith(new WaitCommand(1.1).andThen(intake.runOut())));
    
    // SCORE CONE HIGH
    operatorJoystick.a().onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT)
    ).onFalse(intake.runOut());

    operatorJoystick.button(8).onTrue(new InstantCommand(() -> elevator.resetEncoder()));
    
    // SCORE CUBE MID
    operatorJoystick.button(6).onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CUBE_MID_HEIGHT+550)
    ).onFalse(intake.runOut());
    
    // SCORE CUBE HIGH
    operatorJoystick.button(5).onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CUBE_HIGH_HEIGHT+550)
    ).onFalse(intake.runOut());

    // INTAKE
    operatorJoystick.button(7).onTrue(intake.runIn()).onFalse(intake.stop()); // manual intaking
    // operatorJoystick.button(8).onTrue(intake.runOut()).onFalse(intake.stop()); // manual scoring

    //LEDS TOGGLE
    operatorJoystick.button(10).onTrue(leds.toggleLEDs());
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