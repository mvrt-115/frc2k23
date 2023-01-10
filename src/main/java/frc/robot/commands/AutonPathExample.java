// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutonPathExample extends SequentialCommandGroup {
  /** Creates a new AutonPathExample. */
  private final SwerveDrivetrain swerveDrivetrain;
  private SwerveControllerCommand swerveControllerCommand;
  private Trajectory trajectory;

  public AutonPathExample(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    addRequirements(drivetrain);

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 2),
        new Translation2d(3, 1),
        new Translation2d(5, 4)
      ), 
      new Pose2d(4, 2, Rotation2d.fromDegrees(540.0)),
      swerveDrivetrain.getTrajectoryConfig());
    
    swerveDrivetrain.getField().getObject("traj").setTrajectory(trajectory);

    swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveDrivetrain::getPose, 
      swerveDrivetrain.getKinematics(), 
      swerveDrivetrain.xController, 
      swerveDrivetrain.yController, 
      swerveDrivetrain.thetaController, 
      swerveDrivetrain::setModuleStates,
      swerveDrivetrain);
    
    addCommands(
      new InstantCommand(() -> swerveDrivetrain.setAutonomous()),
      new InstantCommand(() -> swerveDrivetrain.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveDrivetrain.stopModules()),
      new InstantCommand(() -> swerveDrivetrain.setDisabled()));
  }
}
