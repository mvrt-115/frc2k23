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
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.Localization2;
import frc.robot.subsystems.SwerveDrivetrain;

public class Align extends CommandBase {
  private SwerveDrivetrain swerve;
  private Localization2 localization;

  private Pose2d poseToGoTo;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;

  /** Creates a new Align. */
  public Align(SwerveDrivetrain swerve, Localization2 localization) {
    this.swerve = swerve;
    this.localization = localization;
    this.poseToGoTo = localization.getClosestScoringLoc();
    addRequirements(localization, swerve);
    pidX = new PIDController(1.2, 0, 0); // pid x-coor 1.2
    pidY = new PIDController(1, 0, 0.05); // pid y-coor 1.2
    pidTheta = new PIDController(4, 0, 0); // pid t-coor 4
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    localization.setAligning(true);
    localization.resetTemp();
  }

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() {
    Pose2d robotPose = localization.getCurrentPose();
    // if(Localization.distFromTag(robotPose, poseToGoTo) >
    // Constants.VisionConstants.minDistFromTag){
    double outX = pidX.calculate(robotPose.getX(), poseToGoTo.getX())*0.7; // pos, setpoint
    double outY = pidY.calculate(robotPose.getY(), poseToGoTo.getY())*0.7;
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

    // If close enough to target
    // return Math.abs(swerve.getRotation2d().getDegrees() -
    // poseToGoTo.getRotation().getDegrees()) < 10;
    // return Math.abs(robotPose.getY()-poseToGoTo.getY())<0.05 &&
    // Math.abs(robotPose.getX()-poseToGoTo.getX())<0.05 ;
    // return false;//Math.abs(robotPose.getY()-poseToGoTo.getY())<0.05 ;

     return Math.abs(robotPose.getX() - poseToGoTo.getX()) < Constants.VisionConstants.xyTolerance &&
     Math.abs(robotPose.getY() - poseToGoTo.getY()) < Constants.VisionConstants.xyTolerance &&
      Math.abs(swerve.getRotation2d().getDegrees() - poseToGoTo.getRotation().getDegrees()) <
        Constants.VisionConstants.thetaTolerance;
  }

}