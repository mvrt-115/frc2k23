// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.SwerveDrivetrain;
// import frc.robot.utils.BetterSwerveControllerCommand;

public class AutonRunner extends SequentialCommandGroup {
  /** Creates a new AutonPathExample. */
  private final SwerveDrivetrain swerveDrivetrain;
  // private BetterSwerveControllerCommand swerveControllerCommand;
  private PathPlannerTrajectory trajectory;
  private PathConstraints constraints = new PathConstraints(
    Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, 
    Constants.SwerveDrivetrain.kMaxAutonDriveAcceleration);

  public AutonRunner(SwerveDrivetrain drivetrain, Elevator elevator, Intake2 intake, GroundIntake groundIntake, String pathName) {
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

    trajectory = PathPlanner.loadPath(
      pathName, 
      constraints);
    
    swerveDrivetrain.getField().getObject("traj").setTrajectory(trajectory);

    // i
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
      new ParallelCommandGroup(
        new ParallelRaceGroup(new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT-0.25, 0.35),
        new BetterWaitCommand(2)
      ),
      intake.stop()
      ),
      new BetterWaitCommand(0.35),
        intake.runOut(),
      new ElevateDown(elevator))  
    );
    eventMap.put("IntakeDown", new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 1),
      new ParallelCommandGroup(
        new SetGroundIntakePosition(groundIntake, 180),
        new InstantCommand(() -> groundIntake.setRollerOutput(0.2))
      ), 
      new ElevateDown(elevator)
    ));

    eventMap.put("IntakeUp", new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 1),
      new ParallelCommandGroup(
        new SetGroundIntakePosition(groundIntake, 40),
        new InstantCommand(() -> groundIntake.setRollerOutput(0.2))
      ),
      new ElevateDown(elevator)
    ));
    eventMap.put("GIScore", new SequentialCommandGroup(
      new SetElevatorHeight(elevator, 20, 1),
      new ParallelCommandGroup(
        new SetGroundIntakePosition(groundIntake, 120),
        new InstantCommand(() -> groundIntake.setRollerOutput(-0.6))
      ),
      new BetterWaitCommand(0.5),
      new SetGroundIntakePosition(groundIntake, 40),
      new InstantCommand(() -> groundIntake.stopRoller()),
      new ElevateDown(elevator)
    ));

    eventMap.put("LevelForwards", new SequentialCommandGroup(
      new DriveForward(drivetrain, 2.5, 1.25),
      new Leveling(drivetrain))  
    );
    eventMap.put("LevelBackwards", new SequentialCommandGroup(
      new DriveForward(drivetrain, -4, 1.25),
      new Leveling(drivetrain))  
    );

    List<PathPlannerTrajectory> fullTrajectoriesWithStopEvents = PathPlanner.loadPathGroup(
      pathName, 
      constraints);

    SmartDashboard.putString("markers", trajectory.getMarkers().toString());

    // If only markers are not stop events: 
    // FollowPathWithEvents autoEventsCommand = new FollowPathWithEvents(
    //   swerveDrivetrain.getAutonPathCommand(trajectory), 
    //   trajectory.getMarkers(),
    //   eventMap
    // );

    Command autoEventsCommand = swerveDrivetrain.getAutonBuilder(eventMap).fullAuto(fullTrajectoriesWithStopEvents);

    SmartDashboard.putNumber("Trajectory X init", trajectory.getInitialHolonomicPose().getX());
    SmartDashboard.putNumber("Trajectory Y init", trajectory.getInitialHolonomicPose().getY());

    addCommands(
      new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)),
      new InstantCommand(() -> swerveDrivetrain.setAutonomous()),
      new InstantCommand(() -> swerveDrivetrain.resetModules()),
      new InstantCommand(() -> swerveDrivetrain.resetModuleDrive()),
      new InstantCommand(() -> swerveDrivetrain.setModes(NeutralMode.Brake)),
      new InstantCommand(() -> swerveDrivetrain.resetOdometry(trajectory.getInitialHolonomicPose())),
      new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)),
      autoEventsCommand,
      new InstantCommand(() -> swerveDrivetrain.stopModules()),
      new InstantCommand(() -> swerveDrivetrain.setDisabled()));
  }
}