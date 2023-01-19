// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveForward extends CommandBase {

  private SwerveDrivetrain swerveDt;
  private double mps;
  private double time;
  private double currentTime;
  private double startTime;

  /** Creates a new DriveForward. */
  public DriveForward(SwerveDrivetrain swerveDt, double mps, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDt = swerveDt;
    this.mps = mps;
    this.time = time;
    currentTime = 0;
    startTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDt.setSpeeds(mps, 0, 0, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDt.setSpeeds(0, 0, 0, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentTime-startTime >= time)
      return true;
    return false;
  }
}
