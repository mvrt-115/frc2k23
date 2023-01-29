// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Localization;

public class AlignAndExtend extends CommandBase {
  private Align align;
  private SetElevatorHeight setHeight;
  private Elevator elevator;
  private boolean forget; //forget about align command
  private double height;

  //Assumes is already at correct location
  public AlignAndExtend(SwerveDrivetrain swerve, Localization localization, Elevator elevator, Pose2d poseToGoTo, double height) {
    align = new Align(swerve, localization, poseToGoTo);
    this.elevator = elevator;
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!forget && align.isFinished()){
      setHeight = new SetElevatorHeight(elevator, height);
      forget = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (forget && setHeight.isFinished()){
      return true;
    }
    return true;
  }
}