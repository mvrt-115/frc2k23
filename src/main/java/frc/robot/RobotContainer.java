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
    ); // used to 0.4, makes slower speed

    gi.setDefaultCommand(new ManualGroundIntake(gi, () -> operatorJoystick.getRightX()*0.2));
      
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
    driveJoystick.button(3).onTrue(new ResetOdometryWithVision(swerveDrivetrain, localization));
    driveJoystick.button(4).onTrue(new InstantCommand(() -> swerveDrivetrain.resetOdometry(new Pose2d(0,0,new Rotation2d())))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)));

    autonSelector.addOption("ScoreExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, gi, "ExitLevel"));
    autonSelector.addOption("ScoreExitLevel2", new AutonRunner(swerveDrivetrain, elevator, intake, gi, "ExitLevel2"));
    // autonSelector.addOption("ScoreExitLevel", new AutonRunner(swerveDrivetrain, elevator, intake, gi, "ScoreExitLevel"));
    autonSelector.addOption("DONOTHING", new PrintCommand("hi"));
    autonSelector.addOption("ScoreLevel", new SequentialCommandGroup( 
      new AutoScoreCone(elevator, intake), 
      new AutoLevel(swerveDrivetrain, -4)
    ));
    autonSelector.addOption("ScoreTwiceLevel", new AutonRunner(swerveDrivetrain, elevator, intake, gi, "ScoreTwiceLevel"));
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
        new Leveling(swerveDrivetrain) 
      )
    ).onFalse( 
      new InstantCommand(() -> swerveDrivetrain.stopModules())
    );
    
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

    // RETURN TO NEUTRAL
    operatorJoystick.b().onTrue(new SequentialCommandGroup(
      // new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)),
      // new InstantCommand(() -> swerveDrivetrain.stopModules()),
      intake.runOut(),
      new BetterWaitCommand(0.25),
      new ParallelCommandGroup(
        new ElevateDown(elevator),
        intake.stop()
        // new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Coast))
      )
    )).onFalse(
      new InstantCommand(() -> elevator.runMotor(0))
    );

    // gi.setDefaultCommand(new ManualGroundIntake(gi, () -> operatorJoystick.getRightX()*0.2  ));

    // SCORE CONE MID 
    operatorJoystick.y().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT, 0.25));

    // SCORE CONE HIGH
    operatorJoystick.a().onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT, 0.25)
    ).onFalse(intake.runOut());

    // RESET ELEVATOR ENCODER VALUE
     operatorJoystick.button(7).onTrue(new InstantCommand(() -> elevator.resetEncoder()));

    operatorJoystick.button(8).onTrue(new SequentialCommandGroup(
      // new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)),
      // new InstantCommand(() -> swerveDrivetrain.stopModules()),
      intake.runOut(),
      new BetterWaitCommand(0.25),
      new ParallelCommandGroup(
        new ElevateDown(elevator),
        intake.runOutCube()
        // new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Coast))
      )
    )).onFalse(
      new InstantCommand(() -> elevator.runMotor(0))
    );
    
    // SCORE CUBE MID
    operatorJoystick.button(5).onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CUBE_MID_HEIGHT, 0.25)
    );
    
    // SCORE CUBE HIGH
    operatorJoystick.button(6).onTrue(
      new SetElevatorHeight(elevator, Constants.Elevator.CUBE_HIGH_HEIGHT, 0.25)
    );

    operatorJoystick.button(7).onTrue(intake.runOutCube()).onFalse(intake.stop());

    testJoystick.a().onTrue(new SetGroundIntakePosition(gi, 180));
    testJoystick.b().onTrue(new SetGroundIntakePosition(gi, 120));
    testJoystick.x().onTrue(new SetGroundIntakePosition(gi, 40));
    testJoystick.x().onTrue(leds.toggleColor());

    

    // MANUAL INTAKE
  //   operatorJoystick.button(7).onTrue(intake.runIn()).onFalse(intake.stop());
  //  // operatorJoystick.button(7).onTrue(new SetGroundIntakeArmPos(gi, 30));//intake.runIn()).onFalse(intake.stop()); // manual intaking
  //    operatorJoystick.button(8).onTrue(intake.runOut()).onFalse(intake.stop()); // manual scoring

    //LEDS TOGGLE
    // operatorJoystick.button(10).onTrue(leds.toggleLEDs());
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

  public Command getReleaseCommand() {
    if(Math.abs(elevator.getHeightInches() - Constants.Elevator.CONE_MID_HEIGHT) < 3) {
      return (
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT-3, 0.25),
            intake.runOut()
          ),
          new WaitCommand(0.5),
          new ElevateDown(elevator)
        )
      );
    }
    return (
      new SequentialCommandGroup(
        new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)),
        new InstantCommand(() -> swerveDrivetrain.stopModules()),
        intake.runOut(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
          new ElevateDown(elevator),
          intake.stop(),
          new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Coast))
        )
      )
    );
  }
}