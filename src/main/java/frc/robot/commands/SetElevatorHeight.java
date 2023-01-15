// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorHeight extends CommandBase
{
  private Elevator elevator;
  private double height;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator elevator, double height)
  {
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.height = height; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // if(elevator.getLevel() == 3) {
    //   elevator.setTargetHeight(0);
    // }
    // else {
      elevator.setTargetHeight(height);
    //  System.out.println("target height: " + height);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.currentIsWithinError());
  }
}
