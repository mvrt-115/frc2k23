// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends CommandBase {
  /** Creates a new ManualElevator. */
  private Elevator e;
  private Supplier<Double> speed;
  public ManualElevator(Elevator q, Supplier<Double> speed) {
    e = q;
    addRequirements(e);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (e.getHeightInches() < 2)
      e.runMotor(speed.get());
    else
      e.runMotor(speed.get()+Constants.Elevator.kG/10.0);///Constants.Talon.MAX_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
