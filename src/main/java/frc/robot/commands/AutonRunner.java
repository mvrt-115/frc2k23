// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    
    addCommands(
      new InstantCommand(() -> swerveDrivetrain.setAutonomous()),
      new InstantCommand(() -> swerveDrivetrain.resetModules()),
      new InstantCommand(() -> swerveDrivetrain.resetOdometry(trajectory.getInitialHolonomicPose())),
      new InstantCommand(() -> SmartDashboard.putBoolean("Reset Odometry", false)),
      //temp scoring preload mid
      new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT+550),
      new WaitCommand(0.25),
      new ParallelCommandGroup(
        new SetElevatorHeight(elevator, Constants.Elevator.CONE_MID_HEIGHT-3700),
        intake.runOut()
      ),
      new SetElevatorHeight(elevator, 400),
      swerveDrivetrain.getAutonPathCommand(trajectory),
      new DriveForward(drivetrain, -4, 1.25),
      new Leveling(drivetrain),
      new InstantCommand(() -> swerveDrivetrain.stopModules()),
      new InstantCommand(() -> swerveDrivetrain.setDisabled()));
  }
}