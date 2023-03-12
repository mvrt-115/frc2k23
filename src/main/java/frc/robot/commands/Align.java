// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class Align extends CommandBase {
  private SwerveDrivetrain swerve;
  private Localization localization;

  private Supplier<Pose2d> poseSup;
  private Pose2d poseToGoTo;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;

  /** Creates a new Align. */
  public Align(SwerveDrivetrain swerve, Localization localization, Supplier<Pose2d> poseSup) {
    this.swerve = swerve;
    this.localization = localization;
    this.poseSup = poseSup;

    pidX = new PIDController(2.5, 0, 0.2); // pid x-coor 1.2
    pidY = new PIDController(2.5, 0, 0.2); // pid y-coor 1.2
    pidTheta = new PIDController(4, 0, 0); // pid t-coor 4
    
    addRequirements(localization, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    localization.resetCameraEstimators(); //reset estimators before getting the closest scoring location
    localization.setAligning(true);
    poseToGoTo = poseSup.get();
    SmartDashboard.putString("chicken align initialize scoring loc", poseToGoTo.toString());
    SmartDashboard.putString("chicken align currPose", localization.getCurrentPose().toString());
  }

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() {
    Pose2d robotPose = localization.getCurrentPose();
    double outX = -pidX.calculate(robotPose.getX(), poseToGoTo.getX()); // pos, setpoint
    double outY = pidY.calculate(robotPose.getY(), poseToGoTo.getY());
    double outTheta = pidTheta.calculate(localization.normalizeAngle(robotPose.getRotation()).getRadians(),
                         poseToGoTo.getRotation().getRadians());

    SmartDashboard.putString("chicken alliance", DriverStation.getAlliance().toString());                    

    // swerve screwed up field oriented switched y axis; shoud be -outX
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(outX, outY, outTheta,
        new Rotation2d(-robotPose.getRotation().getRadians()));
    SwerveModuleState[] states = swerve.getKinematics().toSwerveModuleStates(speeds);
    swerve.setModuleStates(states);
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

    return Math.abs(robotPose.getX() - poseToGoTo.getX()) < Constants.VisionConstants.xTolerance &&
      Math.abs(robotPose.getY() - poseToGoTo.getY()) < Constants.VisionConstants.yTolerance &&
      Math.abs(robotPose.getRotation().getDegrees() - poseToGoTo.getRotation().getDegrees()) <
      Constants.VisionConstants.thetaTolerance;
  }
}