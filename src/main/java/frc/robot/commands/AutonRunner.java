// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.SwerveDrivetrain;
// import frc.robot.utils.BetterSwerveControllerCommand;

public class AutonRunner extends SequentialCommandGroup {
  /** Creates a new AutonPathExample. */
  private final SwerveDrivetrain swerveDrivetrain;
  // private BetterSwerveControllerCommand swerveControllerCommand;
  private PathPlannerTrajectory trajectory;

  public AutonRunner(SwerveDrivetrain drivetrain, Elevator elevator, Intake2 intake, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    addRequirements(swerveDrivetrain);

    // trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   List.of(
    //     new Translation2d(1, 0),
    //     new Translation2d(1, 1),
    //     new Translation2d(0, 1)
    //   ), 
    //   new Pose2d(0, 2, Rotation2d.fromDegrees(90.0)),
    //   swerveDrivetrain.getTrajectoryConfig());

    trajectory = PathPlanner.loadPath(pathName, 
      new PathConstraints(
        Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, 
        Constants.SwerveDrivetrain.kMaxAutonDriveAcceleration));
    
    swerveDrivetrain.getField().getObject("traj").setTrajectory(trajectory);

    // swerveControllerCommand = new BetterSwerveControllerCommand(
    //   trajectory, 
    //   swerveDrivetrain::getPose, 
    //   swerveDrivetrain.getKinematics(), 
    //   swerveDrivetrain.xController, 
    //   swerveDrivetrain.yController, 
    //   swerveDrivetrain.thetaController,
    //   swerveDrivetrain::setModuleStates,
    //   swerveDrivetrain);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ScoreHigh", new SequentialCommandGroup(
      new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT+550),
      new WaitCommand(0.25),
      new ParallelCommandGroup(
        new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT-3700),
        intake.runOut()
      ),
      new SetElevatorHeight(elevator, 400)
    ));
    eventMap.put("Intake", new PrintCommand("Should Intake Here"));
    eventMap.put("DriveBackwards", new DriveForward(drivetrain, -4, 1.25));
    eventMap.put("DriveForwards", new DriveForward(drivetrain, 4, 1.25));
    eventMap.put("Level", new Leveling(drivetrain));

    FollowPathWithEvents autoEventsCommand = new FollowPathWithEvents(
      swerveDrivetrain.getAutonPathCommand(trajectory), 
      trajectory.getMarkers(),
      eventMap
    );

    addCommands(
      new InstantCommand(() -> swerveDrivetrain.setAutonomous()),
      new InstantCommand(() -> swerveDrivetrain.resetModules()),
      new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)),
      new InstantCommand(() -> swerveDrivetrain.resetOdometry(trajectory.getInitialHolonomicPose())),
      new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)),
      autoEventsCommand,
      new InstantCommand(() -> swerveDrivetrain.stopModules()),
      new InstantCommand(() -> swerveDrivetrain.setDisabled()));
  }
}