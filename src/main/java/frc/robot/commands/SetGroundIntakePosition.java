// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SetGroundIntakePosition extends CommandBase {
  GroundIntake groundIntake;
  double position;
  /** Creates a new RunGroundIntake. */
  public SetGroundIntakePosition(GroundIntake gi, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    groundIntake = gi;
    this.position = position;

    addRequirements(gi);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return Math.abs(position - groundIntake.getArmCurrentPositionDegrees()) < 5;
  }
}
