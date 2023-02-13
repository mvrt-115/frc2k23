// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.MathUtils;

public class SwerveJoystickCommand extends CommandBase {
  private final SwerveDrivetrain drivetrain;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, turnSpeedFunc;
  private final Trigger fieldOrientedFunc;
  private final SlewRateLimiter xLimiter, yLimiter, wLimiter;
  private final JoystickIO joystick;
  private Timer timer;
  private Rotation2d heading;

  public PIDController thetaController;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(SwerveDrivetrain drivetrain, Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc, Supplier<Double> angularSpeedFunc, Trigger fieldOrientedFunc, JoystickIO joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turnSpeedFunc = angularSpeedFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;
    this.xLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    this.wLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kTurnMaxAcceleration);
    this.joystick = joystick;
    thetaController = new PIDController(Constants.JoystickControls.kPJoystick, Constants.JoystickControls.kIJoystick, Constants.JoystickControls.kDJoystick);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setJoystick();
    fieldOrientedFunc.onTrue(new InstantCommand(() -> drivetrain.toggleMode())); //.debounce(0.1);
    timer = new Timer();
    timer.reset();
    timer.start();
    heading = drivetrain.getRotation2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vX = xSpeedFunc.get(); // as of here, negative X is backwards, positive X is forward
    double vY = ySpeedFunc.get(); // as of here, positive Y is left, negative Y is right
    double vW = turnSpeedFunc.get(); // as of here, negative W is down (CW) positive W is up (CCW)
    Logger.getInstance().recordOutput("Controller/vX raw", vX);
    Logger.getInstance().recordOutput("Controller/vY raw", vY);
    Logger.getInstance().recordOutput("Controller/vW raw", vW);

    // apply deadband
    vX = MathUtils.handleDeadband(vX, Constants.SwerveDrivetrain.kThrottleDeadband);
    vY = MathUtils.handleDeadband(vY, Constants.SwerveDrivetrain.kThrottleDeadband);
    vW = MathUtils.handleDeadband(vW, Constants.SwerveDrivetrain.kWheelDeadband);

    // limit acceleration
    vX = xLimiter.calculate(vX) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vY = yLimiter.calculate(vY) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vW = wLimiter.calculate(vW) * Constants.SwerveDrivetrain.kTurnMaxSpeedRPS;

    // configure rotate point
    drivetrain.setRotationPointIdx(0);
    // int POV = joystick.getPOV();
    // switch(POV) {
    //   case 0: drivetrain.setRotationPointIdx(1);
    //   case 90: drivetrain.setRotationPointIdx(2);
    //   case 180: drivetrain.setRotationPointIdx(3);
    //   case 270: drivetrain.setRotationPointIdx(4);
    //   default: drivetrain.setRotationPointIdx(0);
    // }

    SmartDashboard.putBoolean("Field Oriented", drivetrain.fieldOriented);
    boolean isTurnBroken = true;
    // apply heading correction to the robot
    double true_heading = Math.toRadians(drivetrain.getRelativeHeading());
    double desired_heading = MathUtils.betterATanDeg(vX, vY); // deg
    double omega_offset = desired_heading - thetaController.calculate(true_heading, desired_heading);
    SmartDashboard.putNumber("Omega Offset", omega_offset);

    double v_omega = vW;
    if(!isTurnBroken){
      v_omega = vW + omega_offset;
    }

    if (drivetrain.fieldOriented) {
      drivetrain.setSpeedsFieldOriented(vX, vY, v_omega);
    }
    else {
      drivetrain.setSpeeds(vX, vY, v_omega, Constants.SwerveDrivetrain.rotatePoints[0]);
    }
    
    SmartDashboard.putNumber("vX", vX);
    SmartDashboard.putNumber("vY", vY);
    SmartDashboard.putNumber("vW", vW);

    if (MathUtils.withinEpsilon(vX, 0, 0.01) && MathUtils.withinEpsilon(vY, 0, 0.01) && MathUtils.withinEpsilon(vW, 0, 0.01)) {
      drivetrain.stopModules();
      drivetrain.setRotationPointIdx(0);
      if (timer.advanceIfElapsed(1))
      {
        drivetrain.resetModules();
      }
      // drivetrain.holdHeading(heading);
    }
    else {
      timer.reset();
      timer.start();
      heading = drivetrain.getRotation2d();
      drivetrain.thetaController.reset(heading.getRadians());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
    drivetrain.setDisabled();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
