// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class MobileCharge extends CommandBase {
  SwerveDrivetrain swerveDrivetrain;
  boolean onCharge;
  double groundStart;
  double start;
  /** Creates a new MobileCharge. */
  public MobileCharge(SwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;
    addRequirements(swerveDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onCharge = false;
    groundStart = -1;
    start = Timer.getFPGATimestamp();
    swerveDrivetrain.setSpeeds(-3, 0, 0, Constants.SwerveDrivetrain.m_standardCenterLocation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!onCharge && Math.abs(swerveDrivetrain.getRoll()) > 5) {
      onCharge = true;
    }

    if(groundStart == -1 && onCharge && Math.abs(swerveDrivetrain.getRoll()) < 10) {
      groundStart = Timer.getFPGATimestamp();
    }
    if(groundStart != -1 && Math.abs(swerveDrivetrain.getRoll()) > 5)
      groundStart = -1;

    
    //Logger.getInstance().recordOutput("onCharge", onCharge);
   // Logger.getInstance().recordOutput("groundst", groundStart);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (groundStart != -1 && Math.abs(groundStart - Timer.getFPGATimestamp()) > 0.25 && Math.abs(swerveDrivetrain.getRoll()) < 5) || Timer.getFPGATimestamp() - start > 5;
  }
}
