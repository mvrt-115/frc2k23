// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundIntake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class ManualGroundIntake extends CommandBase {
  /** Creates a new SetIntakeArmPosition. */
  private GroundIntake groundIntake;
  private Logger logger;
  private Supplier<Double> out;
  //private boolean remainAtOrigPos;
  public ManualGroundIntake(GroundIntake gi, Supplier<Double> output) {
    // Use addRequirements() here to declare subsystem dependencies.
    groundIntake = gi;
    this.out = output;
    // logger.recordOutput("GroundIntake/armMotor/commandCalled", 1);
    addRequirements(gi);
    //remainAtOrigPos = remainAtPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //remainAtOrigPos = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(!remainAtOrigPos)
    groundIntake.setArmOutput(out.get() + groundIntake.getFeedForward());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
