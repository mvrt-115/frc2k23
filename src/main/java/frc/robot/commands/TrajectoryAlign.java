// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.BetterSwerveControllerCommand;

public class TrajectoryAlign extends SequentialCommandGroup {
  public TrajectoryAlign(SwerveDrivetrain swerveDrivetrain, Localization localization, Pose2d finalLocation) {
    addRequirements(swerveDrivetrain, localization);

    localization.resetCameraEstimators();
    localization.setAligning(true);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      localization.getCurrentPose(), 
      List.of(), 
      finalLocation, 
      swerveDrivetrain.getTrajectoryConfig()
    );

    BetterSwerveControllerCommand swerveControllerCommand = new BetterSwerveControllerCommand(
      trajectory, 
      localization::getCurrentPose, 
      swerveDrivetrain.getKinematics(), 
      swerveDrivetrain.xController, 
      swerveDrivetrain.yController, 
      swerveDrivetrain.thetaController, 
      swerveDrivetrain::setModuleStates,
      swerveDrivetrain
    );

    addCommands(
      swerveControllerCommand,
      new InstantCommand(() -> swerveDrivetrain.stopModules()),
      new InstantCommand(() -> swerveDrivetrain.setDisabled())
    );
  }
}