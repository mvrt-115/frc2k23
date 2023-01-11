// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;

public class Leveling extends CommandBase {
  private SwerveDrivetrain swerveDt;
  private PIDController pid; // Look into ProfilePIDContoller
  private boolean level;

  /** Creates a new Leveling. */
  public Leveling(SwerveDrivetrain _swerveDt) {
    this.swerveDt = _swerveDt;
    this.pid = new PIDController(Constants.SwerveDrivetrain.levelkP, Constants.SwerveDrivetrain.levelkI, Constants.SwerveDrivetrain.levelkD);
    this.level = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("pitch", swerveDt.getPitchAngle());
    SmartDashboard.putNumber("pitch", swerveDt.getLinearVelocity().getNorm());

    pid.calculate(swerveDt.getPitchAngle(), 0);

    // Use setSpeeds to set the speed of the swerve, would set angle and rotation point as 0 and Translation2D(0, 0)
    // might need to change the result of the pid.calculate into an acceptable speed input
    if (Math.abs(swerveDt.getPitchAngle()) < Constants.SwerveDrivetrain.angleTolerance && Math.abs(swerveDt.getLinearVelocity().getNorm()) < Constants.SwerveDrivetrain.speedTolerance){
      level = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // call a command that locks the wheels
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return level; // once the speed of the robot is low enough and the angle is small enough, the command will end 
  }
}
