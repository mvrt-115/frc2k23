// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake;

public class AutonIntake extends CommandBase {
  GroundIntake gi;
  int state = 0;
  double startTime = 0;
  /** Creates a new AutonIntake. */
  public AutonIntake(GroundIntake gi) {
    this.gi = gi;
    addRequirements(gi);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 0) {
      gi.setArmOutput(0.2 + gi.getFeedForward());
      if(gi.getArmCurrentPositionDegrees() > 100) {
      startTime = Timer.getFPGATimestamp();
      state++;}
    }
    else if(state == 1) {
      gi.setRollerOutput(0.2);
      if(Timer.getFPGATimestamp() > startTime + 3)
        state++;
    } else if(state == 2) {
      gi.stopRoller();
      gi.setArmOutput(-0.2 + gi.getFeedForward());
      if(gi.getArmCurrentPositionDegrees() < 40)
        state++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gi.stopRoller();
    gi.setArmOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == 3;
  }
}
