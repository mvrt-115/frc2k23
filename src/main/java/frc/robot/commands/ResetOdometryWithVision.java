// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetOdometryWithVision extends CommandBase {
  public SwerveDrivetrain swerve;
  public Localization localization;
  /** Creates a new ResetOdometryWithVision. */
  public ResetOdometryWithVision(SwerveDrivetrain dt, Localization l) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, l);
    this.swerve = dt;
    this.localization = l;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d curr = localization.getCurrentPose();
    if(curr!=null){
      swerve.resetOdometry(curr);
    }
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
    return true;
  }
}