// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

public class SetElevatorHeight extends CommandBase
{
  private Elevator elevator;
  private double height;
  private double startTime;
  private Logger logger;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator elevator2, double height)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator2;
    this.height = height; 
    logger = Logger.getInstance();
    addRequirements(elevator2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    elevator.setTargetHeight(height, startTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // if(elevator.getLevel() == 3) {
    //   elevator.setTargetHeight(0);
    // }
    // else {
      logger.recordOutput("Elevator/elev/target_height", height);
      elevator.setHeightRaw(height);
//      System.out.println("executeeeee");
    //  System.out.println("target height: " + height);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
   // elevator.keepAtHeight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(elevator.getHeight() -  height) <= 500 && Math.abs(elevator.getVelocity()) <= 100);
  }
}
