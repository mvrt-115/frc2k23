// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdleLEDSystem;
import frc.robot.subsystems.Elevator;

public class ElevateDown extends CommandBase {
  private Elevator elevator;
  private CANdleLEDSystem leds;
  /** Creates a new ElevateDown. */
  public ElevateDown(Elevator e, CANdleLEDSystem l) {
    elevator = e;
    leds = l;
    addRequirements(elevator, leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setAligning(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runMotor(-0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getHeightInches() < 10;
  }
}
