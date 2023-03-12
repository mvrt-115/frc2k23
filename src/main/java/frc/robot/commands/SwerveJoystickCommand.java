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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger; 

public class SwerveJoystickCommand extends CommandBase {
  private final SwerveDrivetrain drivetrain;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, turnSpeedFunc;
  private final Trigger fieldOrientedFunc;
  // private final SlewRateLimiter xLimiter, yLimiter, wLimiter;
  private final JoystickIO joystick;
  private Timer timer;
  private Rotation2d heading;
  private Logger logger;
  private Elevator elevator;

  public PIDController thetaController;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(SwerveDrivetrain drivetrain, Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc, Supplier<Double> angularSpeedFunc, Trigger fieldOrientedFunc, JoystickIO joystick, Elevator elev) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turnSpeedFunc = angularSpeedFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;
    // this.xLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    // this.yLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    // this.wLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kTurnMaxAcceleration);
    this.joystick = joystick;
    thetaController = new PIDController(Constants.JoystickControls.kPJoystick, Constants.JoystickControls.kIJoystick, Constants.JoystickControls.kDJoystick);
    addRequirements(drivetrain);
    logger = Logger.getInstance();
    elevator = elev;
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
    if(!drivetrain.fieldOriented)
      drivetrain.toggleMode();
    drivetrain.resetModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vX = xSpeedFunc.get(); // as of here, negative X is backwards, positive X is forward
    double vY = ySpeedFunc.get(); // as of here, positive Y is left, negative Y is right
    double vW = turnSpeedFunc.get(); // as of here, negative W is down (CW) positive W is up (CCW)
    if(elevator.getHeightInches() > 25) {
      vX *= 0.35;
      vY *= 0.35;
      vW *= 0.35;

    }
    Logger.getInstance().recordOutput("Controller/vX raw", vX);
    Logger.getInstance().recordOutput("Controller/vY raw", vY);
    Logger.getInstance().recordOutput("Controller/vW raw", vW);

    // apply deadband
    vX = MathUtils.handleDeadband(vX, Constants.SwerveDrivetrain.kThrottleDeadband);
    vY = MathUtils.handleDeadband(vY, Constants.SwerveDrivetrain.kThrottleDeadband);
    vW = MathUtils.handleDeadband(vW, Constants.SwerveDrivetrain.kWheelDeadband);

    // check joystick left and right triggers
    double left_trigger = joystick.getRawAxis(Constants.SwerveDrivetrain.kDriveLeftTrigger);
    double right_trigger = joystick.getRawAxis(Constants.SwerveDrivetrain.kDriveRightTrigger);
    if (left_trigger > 0.05) {
      Constants.SwerveDrivetrain.kDriveMaxSpeedMPS = (1 - left_trigger + 0.1) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPSNormal;
      Constants.SwerveDrivetrain.kTurnMaxSpeedRPS = (1 - left_trigger + 0.1) * Constants.SwerveDrivetrain.kTurnMaxSpeedRPSNormal;
      Constants.SwerveDrivetrain.kDriveMaxAcceleration = (1 - (left_trigger / 2)) * Constants.SwerveDrivetrain.kDriveMaxAccelerationNormal;
      Constants.SwerveDrivetrain.kTurnMaxAcceleration = (1 - (left_trigger / 2)) * Constants.SwerveDrivetrain.kTurnMaxAccelerationNormal;
    }
    else if (right_trigger > 0.05) {
      Constants.SwerveDrivetrain.kDriveMaxSpeedMPS = (1 + (right_trigger < 0.75 ? right_trigger : (
        ((Constants.SwerveDrivetrain.kDriveMaxSpeedCap - Constants.SwerveDrivetrain.kDriveMaxSpeedMPSNormal * (1.75))/0.25) * (right_trigger - 1) + Constants.SwerveDrivetrain.kDriveMaxSpeedCap
      ))) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPSNormal;
    }
    else {
      Constants.SwerveDrivetrain.kDriveMaxSpeedMPS = Constants.SwerveDrivetrain.kDriveMaxSpeedMPSNormal;
      Constants.SwerveDrivetrain.kTurnMaxSpeedRPS = Constants.SwerveDrivetrain.kTurnMaxSpeedRPSNormal;
      Constants.SwerveDrivetrain.kDriveMaxAcceleration = Constants.SwerveDrivetrain.kDriveMaxAccelerationNormal;
      Constants.SwerveDrivetrain.kTurnMaxAcceleration = Constants.SwerveDrivetrain.kTurnMaxAccelerationNormal;
    }

    if (elevator.getHeightInches() > 25){
        Constants.SwerveDrivetrain.kDriveMaxAcceleration *= 0.5;
        Constants.SwerveDrivetrain.kTurnMaxAcceleration *= 0.5;
    }

    // limit acceleration
    vX *= Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;//xLimiter.calculate(vX) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vY *=  Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;//yLimiter.calculate(vY) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vW *= Constants.SwerveDrivetrain.kTurnMaxSpeedRPS;//wLimiter.calculate(vW) * Constants.SwerveDrivetrain.kTurnMaxSpeedRPS;

    if (MathUtils.withinEpsilon(vW, 0, 0.01)) {
      double v_w_compensate = drivetrain.holdHeading(heading);
      vW += v_w_compensate;
      SmartDashboard.putBoolean("Holding Heading", true);
    }
    else {
      heading = drivetrain.getRotation2d();
      SmartDashboard.putBoolean("Holding Heading", false);
    }

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
    logger.recordOutput("SwerveModule/Field_Oriented", drivetrain.fieldOriented);

    if (drivetrain.fieldOriented) {
      drivetrain.setSpeedsFieldOriented(vX, vY, vW);
    }
    else {
      drivetrain.setSpeeds(vX, vY, vW, Constants.SwerveDrivetrain.rotatePoints[0]);
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
    }
    else {
      timer.reset();
      timer.start();
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