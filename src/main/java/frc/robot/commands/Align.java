// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class Align extends CommandBase {
  private SwerveDrivetrain swerve;
  private Localization localization;

  private Pose2d poseToGoTo;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;

  /** Creates a new Align. */
  public Align(SwerveDrivetrain swerve, Localization localization, Pose2d poseToGoTo) {
    this.swerve = swerve;
    this.localization = localization;
    this.poseToGoTo = poseToGoTo;
    addRequirements(localization, swerve);
    pidX = new PIDController(2, 0.5, 0); // pid x-coor 1.2
    pidY = new PIDController(2, 0.5, 0.05); // pid y-coor 1.2
    pidTheta = new PIDController(4, 0, 0); // pid t-coor 4
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    localization.resetCameraEstimators(); //reset estimators before getting the closest scoring location
    localization.setAligning(true);
    if(poseToGoTo==null) poseToGoTo = Constants.VisionConstants.kRedScoreCols.get(5);//localization.getClosestScoringLoc();
  }

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() {
    Pose2d robotPose = localization.getCurrentPose();
    // if(Localization.distFromTag(robotPose, poseToGoTo) >
    // Constants.VisionConstants.minDistFromTag){
    double outX = pidX.calculate(robotPose.getX(), poseToGoTo.getX()); // pos, setpoint
    double outY = pidY.calculate(robotPose.getY(), poseToGoTo.getY());
    double outTheta = pidTheta.calculate(swerve.getRotation2d().getRadians(),
                         poseToGoTo.getRotation().getRadians());

    // swerve screwed up field oriented switched y axis; shoud be -outX
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(outX, outY, outTheta,
        new Rotation2d(-swerve.getRotation2d().getRadians()));
    SwerveModuleState[] states = swerve.getKinematics().toSwerveModuleStates(speeds);
    swerve.setModuleStates(states);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    localization.setAligning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d robotPose = localization.getCurrentPose();

      return Math.abs(robotPose.getX() - poseToGoTo.getX()) < Constants.VisionConstants.xyTolerance &&
        Math.abs(robotPose.getY() - poseToGoTo.getY()) < Constants.VisionConstants.xyTolerance &&
        Math.abs(swerve.getRotation2d().getDegrees() - poseToGoTo.getRotation().getDegrees()) <
        Constants.VisionConstants.thetaTolerance;
  }
}