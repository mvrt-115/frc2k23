// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class TrajectoryAlign extends SequentialCommandGroup {
  public TrajectoryAlign(SwerveDrivetrain swerveDrivetrain, Localization localization, Pose2d finalLocation) {
    addRequirements(swerveDrivetrain, localization);

    localization.resetCameraEstimators();
    localization.setAligning(true);

    Pose2d currLoc = localization.getCurrentPose();
    PathPoint start = new PathPoint(currLoc.getTranslation(), currLoc.getRotation(), Rotation2d.fromDegrees(0), swerveDrivetrain.getTranslationSpeedMPS());
    PathPoint end = new PathPoint(finalLocation.getTranslation(), finalLocation.getRotation(), Rotation2d.fromDegrees(0));

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
      new PathConstraints(Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, Constants.SwerveDrivetrain.kDriveMaxAcceleration), 
      start,
      end);

    PPSwerveControllerCommand autoEventsCommand = swerveDrivetrain.getAutonPathCommand(trajectory);

    addCommands(
      autoEventsCommand,
      new InstantCommand(() -> swerveDrivetrain.stopModules())
    );
  }
}