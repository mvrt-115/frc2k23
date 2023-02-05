// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  private TalonFXSimCollection driveMotorSim = null;
  private TalonFXSimCollection turnMotorSim = null;

  // private final AnalogInput absEncoder;
  private final CANCoder absEncoder;
  private final boolean absEncoderReversed;
  private final double absEncoderOffsetRad;
  private final double talonEncoderOffset = 0;

  private SwerveModuleState desiredState;
  private SwerveModulePosition modulePosition;

  private String swerveID;
  
  private Logger logger;

  /** Creates a new SwerveModule. */
  public SwerveModule(String _swerveID, int driveID, int turnID, int encoderID, boolean driveReversed, boolean turnReversed,
      boolean encoderReversed, double encoderOffset, SwerveModulePosition position) {
    driveMotor = TalonFactory.createTalonFX(driveID, driveReversed);
    turnMotor = TalonFactory.createTalonFX(turnID, turnReversed);

    if (Constants.DataLogging.currMode == Constants.DataLogging.Mode.SIM) {
      driveMotorSim = ((TalonFX) driveMotor).getSimCollection();
      turnMotorSim = ((TalonFX) turnMotor).getSimCollection();
    }

    driveMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kP);
    driveMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kI);
    driveMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kD);
    driveMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFF);

    turnMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kPTurn);
    turnMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kITurn);
    turnMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kDTurn);
    turnMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFTurn);

    turnMotor.setNeutralMode(NeutralMode.Brake);

    if (Constants.DataLogging.currMode != Constants.DataLogging.Mode.SIM) {
      absEncoderOffsetRad = encoderOffset;
    } 
    else {
      absEncoderOffsetRad = 0;
    }
    absEncoderReversed = encoderReversed;
    absEncoder = new CANCoder(encoderID);

    desiredState = new SwerveModuleState();

    swerveID = "SwerveModule/" + _swerveID;

    resetEncoders();
    modulePosition = position;

    logger = Logger.getInstance();
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
   * @return the true angle of the swerve module (0 means forward with all screws facing left) CCW is positive
   */
  public double getAbsoluteEncoderRad() {
    double angle = absEncoder.getAbsolutePosition();
    angle = Math.toRadians(angle);
    angle -= absEncoderOffsetRad;
    return angle * (absEncoderReversed ? -1.0 : 1.0);
  }

  /**
   * Get the angle of the wheel based on the talonfx
   * 0 is an arbitary position
   * CCW is positive
   * @return angle in rad
   */
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
    // turnMotor.setSelectedSensorPosition(0);
    // talonEncoderOffset = 0 - MathUtils.radiansToTicks(getAbsoluteEncoderRad(),
    //     Constants.Talon.talonFXTicks, Constants.SwerveModule.gear_ratio_turn);
    turnMotor.setSelectedSensorPosition(MathUtils.radiansToTicks(
      getAbsoluteEncoderRad(), 
      Constants.Talon.talonFXTicks, 
      Constants.SwerveModule.gear_ratio_turn
      ));
  }

  /**
   * set the angle of the turn motor
   * 
   * @param radians
   */
  private void setAngle(double radians) {
    if (Math.abs(
        MathUtils.ticksToRadians(turnMotor.getSelectedSensorPosition(), 2048, Constants.SwerveModule.gear_ratio_turn)
            - radians) < 3 * Math.PI / 180) {
      turnMotor.setIntegralAccumulator(0);
    }
    turnMotor.set(
        ControlMode.Position,
        MathUtils.radiansToTicks(
            radians,
            Constants.Talon.talonFXTicks,
            Constants.SwerveModule.gear_ratio_turn) - talonEncoderOffset);

    if (Constants.DataLogging.currMode == Constants.DataLogging.Mode.SIM) {
      turnMotorSim.setIntegratedSensorRawPosition(
        (int) (MathUtils.radiansToTicks(
            radians,
            Constants.Talon.talonFXTicks,
            Constants.SwerveModule.gear_ratio_turn) - talonEncoderOffset));    
    }
  }

  /**
   * set the velocity of the drive motor
   * 
   * @param v_mps
   */
  private void setVelocity(double v_mps) {
    
    logger.recordOutput(swerveID+"/targetVelocityTicks",  MathUtils.rpmToTicks(
      MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
      Constants.SwerveModule.gear_ratio_drive));

    SmartDashboard.putNumber("tpsIn" + absEncoder.getDeviceID(), MathUtils.rpmToTicks(
        MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
        Constants.SwerveModule.gear_ratio_drive));

    driveMotor.set(
        ControlMode.Velocity,
        MathUtils.rpmToTicks(
            MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
            Constants.SwerveModule.gear_ratio_drive));
    
    if (Constants.DataLogging.currMode == Constants.DataLogging.Mode.SIM) {
      driveMotorSim.setIntegratedSensorVelocity(
        (int) MathUtils.rpmToTicks(
            MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
            Constants.SwerveModule.gear_ratio_drive));
      
      double dt = Timer.getFPGATimestamp() - DriveSimulationData.prevTime;
      dt = Math.min(0.3, dt);
      double position = driveMotor.getSelectedSensorPosition() + MathUtils.rpmToTicks(
        MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
        Constants.SwerveModule.gear_ratio_drive) * 0.1 * dt;
      SmartDashboard.putNumber("raw sim sensor pos " + absEncoder.getDeviceID(), position);
      driveMotorSim.setIntegratedSensorRawPosition(
        (int)position
      );
    }
  }

  /**
   * get the swerve module state (with velocity and rotation)
   * 
   * @return SwerveModuleState
   */
  public SwerveModuleState getState() {
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
   * sets the raw state of the module
   * @param state
   */
  public void setRawState(SwerveModuleState state) {
    logger.recordOutput(swerveID+"/DesiredState", state);
    logger.recordOutput(swerveID+"/Disabled", false);
    logModuleTargetData(state);
    setAngle(state.angle.getRadians());
    setVelocity(state.speedMetersPerSecond);
  }

  /**
   * set the desired state of the module
   * performs optimization based on currents state and velocity
   * @param state
   */
  public void setDesiredState(SwerveModuleState state) {
    logger.recordOutput(swerveID+"/DesiredStatePreOptimized", state);
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      disableModule();
      return;
    }
    SwerveModuleState optimized_state = optimize(state);
    setRawState(optimized_state);
    updatePosition();
  }

  /**
   * Find the optimal angle for the module to go to (prevents it from ever
   * rotating more than 90 degrees at a time)
   * 
   * @param state
   * @return optimal state
   */
  public SwerveModuleState optimize(SwerveModuleState state) {
    double targetAngle = state.angle.getDegrees();
    targetAngle %= 360.0;
    if (targetAngle < 0) {
      targetAngle += 360;
    }
    double currentAngle = getRawEncoderRad() * 180.0 / Math.PI;
    double currentAngleNormalized = currentAngle % 360.0;
    double diff = targetAngle - currentAngleNormalized;
    double targetVelocity = state.speedMetersPerSecond;

    if (90.0 < Math.abs(diff) && Math.abs(diff) < 270.0) {
      double beta = 180.0 - Math.abs(diff);
      beta *= Math.signum(diff);
      targetAngle = currentAngle - beta;
      targetVelocity *= -1;
    } else if (Math.abs(diff) >= 270.0) {
      if (diff < 0)
        targetAngle = currentAngle + (360.0 + diff);
      else
        targetAngle = currentAngle - (360.0 - diff);
    } else {
      targetAngle = currentAngle + diff;
    }

    SwerveModuleState newState = new SwerveModuleState(targetVelocity, new Rotation2d(targetAngle * Math.PI / 180.0));
    return newState;
  }

  /**
   * stop the module from running
   */
  public void disableModule() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(ControlMode.PercentOutput, 0);
    logger.recordOutput(swerveID+"/Disabled", true);
  }

  /**
   * Set mode
   */
  public void setMode(NeutralMode mode){
    driveMotor.setNeutralMode(mode);
    turnMotor.setNeutralMode(mode);
  }

  /**
   * get the swerve module name (useful for SmartDashboard)
   */
  public void logMeasuredData() {
    logger.recordOutput(swerveID+"/CancoderDeg", absEncoder.getAbsolutePosition());
    logger.recordOutput(swerveID+"/OffsetCancoderDeg", getAbsoluteEncoderRad() * 180 / Math.PI);
    logger.recordOutput(swerveID+"/TalonEncoderDeg", getRawEncoderRad() * 180 / Math.PI);
    logger.recordOutput(swerveID+"/CurrentVelocityMPS", getDriveVelocity());
  }

  /**
   * This function is used to log the swerve module state
   * 
   * @param state
   */
  public void logModuleTargetData(SwerveModuleState state){
    logger.recordOutput(swerveID+"/TargetAngleDeg", state.angle.getDegrees());
    logger.recordOutput(swerveID+"/TargetVelocityMPS", state.speedMetersPerSecond);
  }

  /**
   * @return id of swerve module
   */
  public String getSwerveID(){
    return swerveID;
  }

  /**
   * @return log the pre-optimized state
   */
  public SwerveModuleState getLoggingState() {
    double velocity = this.getDriveVelocity();
    double angle = this.getAbsoluteEncoderRad();
    if (velocity < 0) {
      angle += Math.PI;
      angle %= 2.0 * Math.PI;
      velocity *= -1;
    }
    return new SwerveModuleState(velocity, new Rotation2d(angle));
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
    SmartDashboard.putString("Module Positions Values " + absEncoder.getDeviceID(), angle + " rad, " + distance + " meters");
    modulePosition.angle = angle;
    modulePosition.distanceMeters = distance;
  }
}
