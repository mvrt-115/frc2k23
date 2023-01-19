// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class Align extends CommandBase {
  private SwerveDrivetrain swerve;
  private Localization localization;

  private Pose2d scorePose;

  private PIDController pidx;
  private PIDController pidy;
  private PIDController pidt;

  /** Creates a new Align. */
  public Align(SwerveDrivetrain swerve, Localization localization, Pose2d scorePose) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.localization = localization;
    this.scorePose = scorePose;

    pidx = new PIDController(0, 0, 0); // pid x-coor
    pidy = new PIDController(0, 0, 0); // pid y-coor
    pidt = new PIDController(0, 0, 0); // pid t-coor
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = localization.getCurrentPose();
    Pose2d scoringPose = localization.getClosestScoringLoc(robotPose);

    if(dist(robotPose, scoringPose) <= Constants.VisionConstants.minDistFromTag)
      moveToScoringPos(robotPose, scoringPose);
  }

    /**
   * Moves to the scoring column using PID
   * @param robotPose The current robotPose
   * @param scorePose The pose of the scoring column
   */
  public void moveToScoringPos(Pose2d robotPose, Pose2d scorePose) {
    double outX = pidx.calculate(robotPose.getX(), scorePose.getX()); // pos, setpoint
    double outY = pidy.calculate(robotPose.getY(), scorePose.getY()); // pos, setpoint
    double outT = pidt.calculate(robotPose.getRotation().getDegrees(), scorePose.getRotation().getDegrees()); // pos, setpoint

    ChassisSpeeds speeds = new ChassisSpeeds(outX, outY, outT);
    SwerveModuleState[] states = swerve.getKinematics().toSwerveModuleStates(speeds);

    swerve.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d robotPose = localization.getCurrentPose();

    //If close enough to target
    return Math.abs(robotPose.getX() - scorePose.getX()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getY() - scorePose.getY()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getRotation().getDegrees() - scorePose.getRotation().getDegrees()) < Constants.VisionConstants.thetaTolerance;
  }

  private double dist(Pose2d p1, Pose2d p2) {
    return Math.sqrt((p1.getX() - p2.getX()) * (p1.getX() - p2.getX()) + 
        (p1.getY() - p2.getY()) * (p1.getY() - p2.getY()));
  }
}
