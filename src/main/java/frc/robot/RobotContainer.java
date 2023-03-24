// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.JoystickIO;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private Localization localization = new Localization(swerveDrivetrain);
  private CANdleLEDSystem leds = new CANdleLEDSystem();

  GroundIntake gi = new GroundIntake();
  private final JoystickIO driveJoystick = new JoystickIO(Constants.SwerveDrivetrain.kDriveJoystickPort, true, false);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);
  private final CommandXboxController testJoystick = new CommandXboxController(2);
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
    );
      
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
    driveJoystick.button(3).onTrue(new Align(swerveDrivetrain, localization, localization::getCurrentPose));
    driveJoystick.button(4).onTrue(new InstantCommand(() -> swerveDrivetrain.resetOdometry(new Pose2d(0,0,new Rotation2d())))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)));

    // autonSelector.addOption("ScoreExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, gi, leds, "ExitLevel"));
    // autonSelector.addOption("ScoreExitLevel2", new AutonRunner(swerveDrivetrain, elevator, intake, gi, leds,"ExitLevel2"));
    // autonSelector.addOption("DONOTHING", new PrintCommand("hi"));
    // autonSelector.addOption("ScoreLevel", new SequentialCommandGroup( 
    //   new AutoScoreCone(elevator, intake), 
    //   new AutoLevel(swerveDrivetrain, -4, leds)
    // ));
    // autonSelector.addOption("ScoreTwiceLevel", new AutonRunner(swerveDrivetrain, elevator, intake, gi, leds, "ScoreTwiceLevel"));
    autonSelector.setDefaultOption("DONOTHING", new PrintCommand("hi"));
    SmartDashboard.putData("Auton Selector", autonSelector);
  
    //Align to nearest column on click
    driveJoystick.button(6).whileTrue(new Align(swerveDrivetrain, localization, () -> localization.getClosestScoringLoc())).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));

    //SHIFT LEFT
    //driveJoystick.button(-1).whileTrue(new Align(swerveDrivetrain, localization, () -> localization.getLeftScoreLoc())).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));
    
    //SHIFT RIGHT
    //driveJoystick.button(-1).whileTrue(new Align(swerveDrivetrain, localization, () -> localization.getRightScoreLoc())).onFalse(new InstantCommand(() -> swerveDrivetrain.stopModules()));

    // AUTO LEVEL
    driveJoystick.button(2).onTrue(
      new SequentialCommandGroup(
        new DriveForward(swerveDrivetrain, Constants.Leveling.driveForwardMPS, Constants.Leveling.driveForwardTime),
        new Leveling(swerveDrivetrain, leds) 
      )
    ).onFalse( 
      new InstantCommand(() -> swerveDrivetrain.stopModules())
    );

    // GROUND INTAKE DOWN / UP
    driveJoystick.button(6).onTrue(new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 1),
      new SetGroundIntakePosition(gi, 180),
      new InstantCommand(() -> gi.setRollerOutput(0.3)),
      new ElevateDown(elevator)
    )).onFalse(new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 1),
      new SetGroundIntakePosition(gi, 40),
      new InstantCommand(() -> gi.stopRoller())
    ));
    
    //Brake baby brake
    //driveJoystick.button(5).onTrue(new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)));

    //No braking
    //driveJoystick.button(6).onTrue(new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Coast)));

    // HP INTAKE
    operatorJoystick.x().onTrue(
      new IntakeHPStation(elevator, intake)
    ).onFalse(
      new ElevateDown(elevator).alongWith(intake.stop())
    );

    // SHOOT CONE AND DOWN
    operatorJoystick.b().onTrue(new SequentialCommandGroup(
      intake.runOut(),
      new BetterWaitCommand(0.25),
      new ParallelCommandGroup(
        new ElevateDown(elevator),
        intake.stop()
      )
    )).onFalse(
      new InstantCommand(() -> elevator.runMotor(0))
    );

    // ELEV MID 
    operatorJoystick.y().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT, 0.25));

    // ELEV HIGH
    operatorJoystick.a().onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT, 0.25)
    ).onFalse(intake.runOut());

    // SHOOT CUBE AND DOWN
    operatorJoystick.rightBumper().onTrue(new SequentialCommandGroup(
      intake.runOutCube(),
      new BetterWaitCommand(0.25),
      new ParallelCommandGroup(
        new ElevateDown(elevator),
        intake.stop()
      )
    )).onFalse(
      new InstantCommand(() -> elevator.runMotor(0))
    );  

    // MANUAL INTAKE
    operatorJoystick.start().onTrue(intake.runIn()).onFalse(intake.stop());

    // MANUAL OUTTAKE
    operatorJoystick.back().onTrue(intake.runOutCube()).onFalse(intake.stop());

    // GROUND INTAKE SHOOT LOW
    operatorJoystick.leftTrigger().onTrue(new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 1),
      new SetGroundIntakePosition(gi, 120),
      new InstantCommand(() -> gi.setRollerOutput(-0.8)),
      new ElevateDown(elevator)
    )).onFalse(new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 0),
      new SetGroundIntakePosition(gi, 40),
      new InstantCommand(() -> gi.stopRoller())
    ));

    // LED TOGGLE
    operatorJoystick.leftBumper().onTrue(new SetLEDCC(leds));

    // RESET ELEVATOR ENCODER VALUE
    operatorJoystick.rightStick().onTrue(new InstantCommand(() -> elevator.resetEncoder()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autonSelector.getSelected();
    return new AutonRunner(swerveDrivetrain, elevator, intake, gi, leds, localization, "ExitRed");
  }

  public void putTestCommand() {
    swerveDrivetrain.setupTests();
  }
}