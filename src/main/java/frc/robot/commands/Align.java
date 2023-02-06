// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    pidX = new PIDController(1.2, 0, 0); // pid x-coor 1.2
    pidY = new PIDController(1.2, 0, 0); // pid y-coor 1.2
    pidTheta = new PIDController(5, 0, 0); // pid t-coor 5
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    localization.setAligning(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = localization.getCurrentPose();
    double outX = pidX.calculate(robotPose.getX(), poseToGoTo.getX())*0.6;
    double outY = pidY.calculate(robotPose.getY(), poseToGoTo.getY())*0.7;

    double outTheta = pidTheta.calculate(robotPose.getRotation().getRadians(), poseToGoTo.getRotation().getRadians());
  
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-outX, outY, outTheta, new Rotation2d(-robotPose.getRotation().getRadians()));
    SwerveModuleState[] states = swerve.getKinematics().toSwerveModuleStates(speeds);
    swerve.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    localization.setAligning(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d robotPose = localization.getCurrentPose();

    //If close enough to target
    return Math.abs(robotPose.getRotation().minus(poseToGoTo.getRotation()).getRadians())<0.1;
    //return Math.abs(robotPose.getY()-poseToGoTo.getY())<0.05 && Math.abs(robotPose.getX()-poseToGoTo.getX())<0.05 ;
    /* 
    return Math.abs(robotPose.getX() - poseToGoTo.getX()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getY() - poseToGoTo.getY()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getRotation().getDegrees() - poseToGoTo.getRotation().getDegrees()) < Constants.VisionConstants.thetaTolerance;*/
  }


}
