// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.LocalizationTest;
import frc.robot.subsystems.SwerveDrivetrain;

public class MyAlign extends CommandBase {
  /** Creates a new MyAlign. */
  SwerveDrivetrain swerve;
  LocalizationTest localization;
  Supplier<Pose2d> supplier;

  PIDController pidX = new PIDController(2.7, 0, 0);
  PIDController pidY = new PIDController(2.7, 0, 0);
  PIDController pidTheta = new PIDController(4, 0, 0);

  public MyAlign(SwerveDrivetrain s, LocalizationTest l, Supplier<Pose2d> p) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = s;
    localization = l;
    supplier = p;
    addRequirements(swerve, localization);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d curLoc = localization.getCurrentPosition();
    Pose2d targetLoc = localization.getTargetPosition();
    double xVel = pidX.calculate(curLoc.getX(), targetLoc.getX());
    double yVel = pidY.calculate(curLoc.getY(), targetLoc.getY());
    double thetaVel = pidTheta.calculate(curLoc.getRotation().getRadians(), targetLoc.getRotation().getRadians());
    swerve.setSpeedsFieldOriented(xVel, yVel, thetaVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d curLoc = localization.getCurrentPosition();
    Pose2d targetLoc = localization.getTargetPosition();
    if (Math.abs(curLoc.getY() - targetLoc.getY()) <= 0.01 && Math.abs(curLoc.getX() - targetLoc.getX()) <= 0.01 && Math.abs(curLoc.getRotation().getRadians() - targetLoc.getRotation().getRadians()) <= 0.05) {
      return true;
    }
    return false;
  }
}
