// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

public class SwerveModule {
  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  // private final AnalogInput absEncoder;
  private final CANCoder absEncoder;
  private final boolean absEncoderReversed;
  private final double absEncoderOffsetRad;
  private double talonEncoderOffset;

  private SwerveModuleState desiredState;
  private SwerveModulePosition modulePosition;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int turnID, int encoderID, boolean driveReversed, boolean turnReversed,
      boolean encoderReversed, double encoderOffset, SwerveModulePosition position) {
    driveMotor = TalonFactory.createTalonFX(driveID, driveReversed);
    turnMotor = TalonFactory.createTalonFX(turnID, turnReversed);

    driveMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kP);
    driveMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kI);
    driveMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kD);
    driveMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFF);

    turnMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kPTurn);
    turnMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kITurn);
    turnMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kDTurn);
    turnMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFTurn);

    turnMotor.setNeutralMode(NeutralMode.Brake);

    absEncoderReversed = encoderReversed;
    absEncoderOffsetRad = encoderOffset;
    absEncoder = new CANCoder(encoderID);

    desiredState = new SwerveModuleState();

    resetEncoders();
    modulePosition = position;
  }

  /**
   * Get the drive position of the module
   * 
   * @return the position in meters
   */
  public double getDrivePosition() {
    return MathUtils.ticksToMeter(
        driveMotor.getSelectedSensorPosition(),
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_drive,
        Constants.SwerveModule.radius);
  }

  /**
   * Get the turn position for the module
   * 
   * @return the position in radians
   */
  public double getTurnPosition() {
    return MathUtils.ticksToRadians(
        turnMotor.getSelectedSensorPosition(),
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_turn);
  }

  /**
   * Get the linear velocity of the module
   * 
   * @return the linear velocity in mps
   */
  public double getDriveVelocity() {
    return MathUtils.rpmToMPS(
        MathUtils.ticksToRPM(
            driveMotor.getSelectedSensorVelocity(),
            Constants.Talon.talonFXTicks,
            Constants.SwerveModule.gear_ratio_drive),
        Constants.SwerveModule.radius);
  }

  /**
   * Get the angular velocity of the module
   * 
   * @return the angular velocity in rpm
   */
  public double getTurnVelocity() {
    return MathUtils.ticksToRPM(
        turnMotor.getSelectedSensorVelocity(),
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_drive);
  }

  /**
   * Get the angle of the absolute encoder sensor on the module
   * 
   * @return the absolute encoder angle in radians
   */
  public double getAbsoluteEncoderRad() {
    double angle = absEncoder.getAbsolutePosition();
    angle = Math.toRadians(angle);
    // absEncoder.getVoltage() / RobotController.getVoltage5V();
    // angle *= 2.0 * Math.PI;
    angle -= absEncoderOffsetRad;
    return angle * (absEncoderReversed ? -1.0 : 1.0);
  }

  public double getRawEncoderRad() {
    double angle = turnMotor.getSelectedSensorPosition();
    angle += talonEncoderOffset;
    return MathUtils.ticksToRadians(
      angle,
      Constants.Talon.talonFXTicks,
      Constants.SwerveModule.gear_ratio_turn);
  }

  /**
   * reset the encoders of the module
   * calibrate turn motor using abs encoder value
   */
  public void resetEncoders() {
    turnMotor.setSelectedSensorPosition(0);
    talonEncoderOffset = 0 - MathUtils.radiansToTicks(getAbsoluteEncoderRad(),
        Constants.Talon.talonFXTicks, Constants.SwerveModule.gear_ratio_turn);
  }

  /**
   * set the angle of the turn motor
   * 
   * @param radians
   */
  public void setAngle(double radians) {
    turnMotor.set(
        ControlMode.Position,
        MathUtils.radiansToTicks(
            radians,
            Constants.Talon.talonFXTicks,
            Constants.SwerveModule.gear_ratio_turn) - talonEncoderOffset);

    SmartDashboard.putNumber("Turn Value " + getName(), MathUtils.radiansToTicks(
        radians,
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_turn));
  }

  /**
   * set the velocity of the drive motor
   * 
   * @param v_mps
   */
  public void setVelocity(double v_mps) {
    driveMotor.set(
        ControlMode.Velocity,
        MathUtils.rpmToTicks(
            MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
            Constants.SwerveModule.gear_ratio_drive));
  }

  /**
   * get the swerve module state (with velocity and rotation)
   * 
   * @return SwerveModuleState
   */
  public SwerveModuleState getState() {
    SmartDashboard.putNumber("TurnMotor " + absEncoder.getDeviceID(), MathUtils.ticksToDegrees(
        turnMotor.getSelectedSensorPosition(), Constants.Talon.talonFXTicks, Constants.SwerveModule.gear_ratio_turn));
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  /**
   * get the desired state of the module
   * 
   * @return SwerveModuleState
   */
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * set the desired state of the module
   * 
   * @param state
   */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      disableModule();
      return;
    }
    state = optimize(state);
    SmartDashboard.putString("Swerve [" + absEncoder.getDeviceID() + "] desired state", state.toString());
    updatePosition();
  }

  /**
   * Find the optimal angle for the module to go to (prevents it from ever rotating more than 90 degrees at a time)
   * @param state
   * @return optimal state
   */
  public SwerveModuleState optimize(SwerveModuleState state) {
      double targetAngle = state.angle.getDegrees();
      targetAngle %= 360.0;
      double currentAngle = getRawEncoderRad() * 180.0 / Math.PI;
      double currentAngleNormalized = currentAngle % 360.0;
      double diff = targetAngle - currentAngleNormalized;
      double velocity = state.speedMetersPerSecond;

      if (90.0 < Math.abs(diff) && Math.abs(diff) < 270.0) {
        double beta = 180.0 - Math.abs(diff);
        beta *= Math.signum(diff);
        targetAngle = currentAngle - beta;
        velocity *= -1;
      }
      else if (Math.abs(diff) >= 270.0) {
        if (diff < 0)
          targetAngle = currentAngle + (360.0 + diff);
        else
          targetAngle = currentAngle - (360.0 - diff);
      }
      else {
        targetAngle = currentAngle + diff;
      }
      
      SmartDashboard.putNumber("Target Angle " + absEncoder.getDeviceID(), targetAngle);
      setAngle(targetAngle * Math.PI / 180.0);
      setVelocity(velocity);
      SwerveModuleState newState = new SwerveModuleState(velocity, new Rotation2d(targetAngle * Math.PI / 180.0));
      return newState;
  }

  /**
   * stop the module from running
   */
  public void disableModule() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(ControlMode.PercentOutput, 0);
    // desiredState = new SwerveModuleState(0, new Rotation2d(0.0));
  }

  /**
   * get the swerve module name (useful for SmartDashboard)
   * 
   * @return String name
   */
  public String getName() {
    SmartDashboard.putNumber("Cancoder " + absEncoder.getDeviceID(), absEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Actual " + absEncoder.getDeviceID(), getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Val in Rad " + absEncoder.getDeviceID(),
        MathUtils.radiansToTicks(getAbsoluteEncoderRad(), Constants.Talon.talonFXTicks,
            Constants.SwerveModule.gear_ratio_turn) / (2048 / Constants.SwerveModule.gear_ratio_turn));
    return "Swerve " + absEncoder.getDeviceID();
  }

  /**
   * get a summary of the modules values
   * 
   * @return
   */
  public String getStateSummary() {
    StringBuilder str = new StringBuilder();
    str.append("Swerve Module: [" + absEncoder.getDeviceID() + "]: ");
    str.append(getState().toString());
    str.append("; Cancoder Position: " + absEncoder.getAbsolutePosition() + "");
    return str.toString();
  }

  /**
   * After reset, zero the wheels
   */
  public void zeroPosition() {
    setAngle(0);
  }

  /**
   * Update the module position for kinematics/odometry
   */
  public void updatePosition() {
    Rotation2d angle = new Rotation2d(getTurnPosition());
    double distance = getDrivePosition();
    modulePosition.angle = angle;
    modulePosition.distanceMeters = distance;
  }
}
