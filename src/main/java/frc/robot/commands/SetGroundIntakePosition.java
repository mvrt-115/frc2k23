// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SetGroundIntakePosition extends CommandBase {
  GroundIntake groundIntake;
  double position;
  double maxRunTime;
  /** Creates a new RunGroundIntake. */
  public SetGroundIntakePosition(GroundIntake gi, double position, double maxRunTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    groundIntake = gi;
    this.position = position;
    this.maxRunTime = maxRunTime;

    addRequirements(gi);
  }

  public SetGroundIntakePosition(GroundIntake gi, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    groundIntake = gi;
    this.position = position;
    this.maxRunTime = 2;

    addRequirements(gi);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    maxRunTime += Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundIntake.setPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() > maxRunTime)
      return true;
    return Math.abs(position - groundIntake.getArmCurrentPositionDegrees()) < 5;
  }
}
