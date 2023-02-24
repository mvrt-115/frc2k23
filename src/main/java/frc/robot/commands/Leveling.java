// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;

public class Leveling extends CommandBase {
  private SwerveDrivetrain swerveDt;
  private PIDController pidLevel; 
  private PIDController pidRotate;
  private boolean level;

  /** Creates a new Leveling. */
  public Leveling(SwerveDrivetrain _swerveDt) {
    this.swerveDt = _swerveDt;
    this.pidLevel = new PIDController(Constants.SwerveDrivetrain.levelkP, Constants.SwerveDrivetrain.levelkI, Constants.SwerveDrivetrain.levelkD);
    this.pidRotate = new PIDController(Constants.SwerveDrivetrain.rotatekP, Constants.SwerveDrivetrain.rotatekI, Constants.SwerveDrivetrain.rotatekD);
    this.level = false;

    addRequirements(swerveDt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pidRotate.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentPitch = swerveDt.getPitchAngle();
    double currentYaw = swerveDt.getYaw();
    double currentRoll = swerveDt.getRoll();

    // SmartDashboard.putNumber("pitch", currentPitch);
    // SmartDashboard.putNumber("yaw", currentYaw);
    // SmartDashboard.putNumber("roll", currentRoll);

    SmartDashboard.putNumber("velocity", swerveDt.getLinearVelocity().getNorm());

    double vX = -Math.min(-currentPitch*Constants.SwerveDrivetrain.levelkP, 1) * Constants.SwerveDrivetrain.levelVelocityMPS;
    double vXpid = pidLevel.calculate(currentPitch, 0) * Constants.SwerveDrivetrain.levelVelocityMPS;
    double angularSpeed = pidRotate.calculate(currentYaw, 0) * Constants.SwerveDrivetrain.maxAngularSpeed;
    double vY = 0;

    swerveDt.setSpeeds(vXpid, vY, angularSpeed, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // call a command that locks the wheels
    swerveDt.setSpeeds(0, 0, 0, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(swerveDt.getPitchAngle()) < Constants.SwerveDrivetrain.angleTolerance && Math.abs(swerveDt.getLinearVelocity().getNorm()) < Constants.SwerveDrivetrain.speedTolerance){
      level = false; //true;
    }
    return level; // once the speed of the robot is low enough and the angle is small enough, the command will end 
  }
}
