// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SetYaw extends CommandBase {

  SwerveDrivetrain swerveDt;
  double yawTarget;
  PIDController pid = new PIDController(Constants.Leveling.rotatekP, Constants.Leveling.rotatekI, Constants.Leveling.rotatekD);

  /** Creates a new SetYaw. */
  public SetYaw(SwerveDrivetrain swerveDt, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDt = swerveDt;
    this.yawTarget = targetAngle;
    addRequirements(swerveDt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angularSpeed = pid.calculate(swerveDt.getYaw(), yawTarget) * Constants.Leveling.maxAngularSpeed;
    swerveDt.setSpeeds(0, 0, angularSpeed, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDt.setSpeeds(0, 0, 0, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(swerveDt.getYaw() - yawTarget) <= Constants.Leveling.angleTolerance);
  }
}
