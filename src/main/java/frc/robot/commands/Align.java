// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class Align extends CommandBase {
  private SwerveDrivetrain swerve;
  private Localization localization;

  private Pose2d scorePose;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;

  /** Creates a new Align. */
  public Align(SwerveDrivetrain swerve, Localization localization) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.localization = localization;
    this.scorePose = localization.getClosestScoringLoc();

    pidX = new PIDController(0, 0, 0); // pid x-coor
    pidY = new PIDController(0.5, 0, 0); // pid y-coor
    pidTheta = new PIDController(0, 0, 0); // pid t-coor
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
    // SmartDashboard
    SmartDashboard.putNumber("scoring x", scorePose.getX());
    SmartDashboard.putNumber("scoring y", scorePose.getY());
    if(localization.distFromTag(robotPose, scorePose) < Constants.VisionConstants.maxDistFromTag){
      double outX = pidX.calculate(robotPose.getX(), scorePose.getX()); // pos, setpoint
      double outY = -pidY.calculate(robotPose.getY(), scorePose.getY()); // pos, setpoint
      double outTheta = pidTheta.calculate(robotPose.getRotation().getRadians(), scorePose.getRotation().getRadians());
      ChassisSpeeds speeds = new ChassisSpeeds(outX, outY, outTheta);
      SwerveModuleState[] states = swerve.getKinematics().toSwerveModuleStates(speeds);
      swerve.setModuleStates(states);
      
      //Log random stuff
      SmartDashboard.putNumber("distance from final", localization.distFromTag(robotPose, scorePose));
      SmartDashboard.putNumber("error x", robotPose.getX() - scorePose.getX());
      SmartDashboard.putNumber("error y", robotPose.getY() - scorePose.getY());
    }
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
    return Math.abs(robotPose.getX() - scorePose.getX()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getY() - scorePose.getY()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getRotation().getDegrees() - scorePose.getRotation().getDegrees()) < Constants.VisionConstants.thetaTolerance;
  }
}
