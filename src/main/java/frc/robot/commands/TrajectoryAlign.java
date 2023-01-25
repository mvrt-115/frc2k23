// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class TrajectoryAlign extends CommandBase {
  
  /** Creates a new TrajectoryAlign. */
  private SwerveDrivetrain swerveDrivetrain;
  private Localization localization;
  private Pose2d finalPos;
  private TrajectoryConfig config;
  private SwerveControllerCommand swerveControllerCommand;
  public TrajectoryAlign(SwerveDrivetrain swerveDrivetrain, Localization localization, Pose2d finalLocation) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.finalPos = finalLocation;
    this.localization = localization;
    config = new TrajectoryConfig(0,0);
    addRequirements(swerveDrivetrain, localization);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}