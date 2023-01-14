// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.SwerveDrivetrain;

public class Align extends CommandBase {
  private SwerveDrivetrain swerve;
  private Localization localization;

  private Pose3d scorePose;

  private PIDController pidx;
  private PIDController pidy;
  private PIDController pidt;

  /** Creates a new Align. */
  public Align(SwerveDrivetrain swerve, Localization localization, Pose3d scorePose) {
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
    Pose3d robotPose = localization.getEstimatedPose();

    moveToScoringPos(robotPose, localization.getClosestScoringLoc(robotPose));
  }

    /**
   * Moves to the scoring column using PID
   * @param robotPose The current robotPose
   * @param scorePose The pose of the scoring column
   */
  public void moveToScoringPos(Pose3d robotPose, Pose3d scorePose) {
    double outX = pidx.calculate(robotPose.getX(), scorePose.getX()); // pos, setpoint
    double outY = pidy.calculate(robotPose.getY(), scorePose.getY()); // pos, setpoint
    double outT = pidt.calculate(robotPose.getRotation().getAngle(), scorePose.getRotation().getAngle()); // pos, setpoint

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
    Pose3d robotPose = localization.getEstimatedPose();

    return Math.abs(robotPose.getX() - scorePose.getX()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getY() - scorePose.getY()) < Constants.VisionConstants.xyTolerance && 
      Math.abs(robotPose.getRotation().getAngle() - scorePose.getRotation().getAngle()) < Constants.VisionConstants.thetaTolerance;
  }
}
