// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. Aman :)

package frc.robot.subsystems;

import java.sql.Time;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase {
  // state stuff
  public static enum DrivetrainState {
    AUTON_PATH, JOYSTICK_DRIVE, DISABLED
  }

  private DrivetrainState state;

  // kinematics stuff
  private SwerveDriveKinematics swerveKinematics;
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModule[] motors;
  private int rotationPoint = 0;
  public boolean fieldOriented = false;

  // Auton Stuff
  private SwerveDriveOdometry odometry;
  private Field2d field;
  public PIDController xController;
  public PIDController yController;
  public ProfiledPIDController thetaController;
  private TrajectoryConfig trajectoryConfig;
  // sensors
  private AHRS gyro;
  // private PigeonIMU gyro;
  private double gyroOffset = 0; // degrees

  private Logger logger;
  
  private DriveSimulationData dsdODE; 
  
  /** Creates a new SwerveDrive. */
  public SwerveDrivetrain() {
    logger = Logger.getInstance();

    gyro = new AHRS(SPI.Port.kMXP);

    // reset in new thread since gyro needs some time to boot up and we don't 
    // want to interfere with other code
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      }
      catch (Exception e)
      {
        System.out.println("Reset Gyro Failed");
      }
    }).start(); 

    swerveKinematics = new SwerveDriveKinematics(
      Constants.SwerveDrivetrain.m_frontLeftLocation, 
      Constants.SwerveDrivetrain.m_frontRightLocation, 
      Constants.SwerveDrivetrain.m_backLeftLocation, 
      Constants.SwerveDrivetrain.m_backRightLocation);
    
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    motors = new SwerveModule[4];

    motors[0] = new SwerveModule(
      "FrontLeft",
      Constants.SwerveDrivetrain.m_frontLeftDriveID,
      Constants.SwerveDrivetrain.m_frontLeftTurnID,
      Constants.SwerveDrivetrain.m_frontLeftEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_frontLeftEncoderOffset,
      modulePositions[0]);

    motors[2] = new SwerveModule(
      "FrontRight",
      Constants.SwerveDrivetrain.m_frontRightDriveID,
      Constants.SwerveDrivetrain.m_frontRightTurnID,
      Constants.SwerveDrivetrain.m_frontRightEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_frontRightEncoderOffset,
      modulePositions[1]);

    motors[1] = new SwerveModule(
      "BackLeft",
      Constants.SwerveDrivetrain.m_backLeftDriveID,
      Constants.SwerveDrivetrain.m_backLeftTurnID,
      Constants.SwerveDrivetrain.m_backLeftEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_backLeftEncoderOffset,
      modulePositions[2]);

    motors[3] = new SwerveModule(
      "BackRight",
      Constants.SwerveDrivetrain.m_backRightDriveID,
      Constants.SwerveDrivetrain.m_backRightTurnID,
      Constants.SwerveDrivetrain.m_backRightEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_backRightEncoderOffset,
      modulePositions[3]);

    odometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d(), modulePositions);
    field = new Field2d();

    xController = new PIDController(Constants.SwerveDrivetrain.m_x_control_P, Constants.SwerveDrivetrain.m_x_control_I, Constants.SwerveDrivetrain.m_x_control_D);
    yController = new PIDController(Constants.SwerveDrivetrain.m_y_control_P, Constants.SwerveDrivetrain.m_y_control_I, Constants.SwerveDrivetrain.m_y_control_D);
    thetaController = new ProfiledPIDController(
      Constants.SwerveDrivetrain.m_r_control_P, 
      Constants.SwerveDrivetrain.m_r_control_I, 
      Constants.SwerveDrivetrain.m_r_control_D, 
      Constants.SwerveDrivetrain.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    trajectoryConfig = new TrajectoryConfig(
      Constants.SwerveDrivetrain.kDriveMaxSpeedMPS, 
      Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    trajectoryConfig.setKinematics(swerveKinematics);
    state = DrivetrainState.JOYSTICK_DRIVE; 
    dsdODE = new DriveSimulationData(new SwerveDriveOdometry(swerveKinematics, new Rotation2d(), modulePositions), field);
  }

  public SwerveModulePosition[] getModulePositions(){
    return modulePositions;
  }
  
  /**
   * Zero the physical gyro
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * get the heading of the physical gyro
   * @return heading angle in degrees
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw() - gyroOffset, 360);
    //.getAngle()

    /**
     * use IEEEremainder because it uses formula:
     * dividend - (divisor x Math.Round(dividend / divisor))
     * versus the remainder operator (%) which uses:
     * (Math.Abs(dividend) - (Math.Abs(divisor) x (Math.Floor(Math.Abs(dividend) / Math.Abs(divisor))))) x Math.Sign(dividend)
     */
  }

  /**
   * get the relative heading of the robot based off of the chassis speeds
   * @return relative angle in degrees
   */
  public double getRelativeHeading() {
    SwerveModuleState[] states = getOutputModuleStates();

    logger.recordOutput("Current SwerveDriveState [LF, LB, RF, RB]", states);

    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(states);
    double angle = Math.atan(speeds.vyMetersPerSecond/speeds.vxMetersPerSecond);
    return Math.toDegrees(angle);
  }

  /**
   * get the gyro angle as rotation2d
   * @return Rotation2d heading
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * update simulation, drive and smart dashboard data
   */
  @Override
  public void periodic() {
    Logger.getInstance().recordOutput("SwerveModuleStatesTrue", new double[]{
      motors[0].getAbsoluteEncoderRad(), motors[0].getDriveVelocity(),
      motors[1].getAbsoluteEncoderRad(), motors[1].getDriveVelocity(),
      motors[2].getAbsoluteEncoderRad(), motors[2].getDriveVelocity(),
      motors[3].getAbsoluteEncoderRad(), motors[3].getDriveVelocity(),
    });
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Robot Heading", getHeading()); //i don't think we need to know this
    SmartDashboard.putData("Field", field);
    // for (SwerveModule m : motors) {
    //   SmartDashboard.putString(m.getName(), m.getStateSummary());
    // }
    // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    // SmartDashboard.putNumber("xvel", getLinearVelocity().getX());
    // SmartDashboard.putNumber("yvel", getLinearVelocity().getY());

    // logger.recordOutput("Robot Heading", getHeading());

    odometry.update(getRotation2d(), modulePositions);

    if(Constants.DataLogging.currMode != Constants.DataLogging.Mode.SIM){
      field.setRobotPose(
        odometry.getPoseMeters().getX(),
        odometry.getPoseMeters().getY(),
        getRotation2d()
      );
    }

    logger.recordOutput("Robot Location", getPose());
  }

  public void simulationPeriodic() {
    dsdODE.quadrature(swerveKinematics.toChassisSpeeds(getOutputModuleStates()).omegaRadiansPerSecond, modulePositions);
  }

  /**
   * stop the swerve modules
   */
  public void stopModules() {
    for (SwerveModule m : motors) {
      m.disableModule();
    }
  }

  /**
   * set the states of all four modules
   * @param states
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrivetrain.kMaxSpeedMPS);
    for (int i = 0; i < motors.length; i++)
    {
      motors[i].setDesiredState(states[i]);
    }
  }

  /**
   * set the states of all four modules using speeds (horizontal, vertical and angular) as well as a rotation point
   * @param v_forwardMps
   * @param v_sideMps
   * @param v_rot (rad/sec)
   * @param rotatePoint
   */
  public void setSpeeds(double v_forwardMps, double v_sideMps, double v_rot, Translation2d rotatePoint) {
    ChassisSpeeds speeds = new ChassisSpeeds(v_forwardMps, v_sideMps, v_rot);
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds, rotatePoint);
    for (int i = 0; i < 4; i++)
    {
      motors[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * get the swerve kinematics of the robot
   * @return SwerveDriveKinematics kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  /**
   * get the linear velocity of the robot in mps
   * @return linear velocity
   */
  public Translation2d getLinearVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getState(),
        motors[1].getState(),
        motors[2].getState(),
        motors[3].getState()
    );
    return new Translation2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond
    );
  }

  /**
   * get the rotational velocity of the robot in rad/sec
   * @return rotational velocity
   */
  public double getRotationalVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getState(),
        motors[1].getState(),
        motors[2].getState(),
        motors[3].getState()
    );
    return speeds.omegaRadiansPerSecond;
  }

  /**
   * get the desired rotational velocity of the robot
   * @return desired rotational velocity rad/sec
   */
  public double getDesiredRotationalVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getDesiredState(),
        motors[1].getDesiredState(),
        motors[2].getDesiredState(),
        motors[3].getDesiredState()
    );
    return speeds.omegaRadiansPerSecond;
  }

  /**
   * get the actual states of all swerve modules
   * @return SwerveModuleState[] states
   */
  public SwerveModuleState[] getOutputModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++)
    {
      states[i] = motors[i].getState();
    }
    return states;
  }

  /**
   * get the position of the robot
   * @return Pose2d pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * reset the real odometry of the robot
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), modulePositions, pose);
  }

  /**
   * get the field
   * @return Field2d field
   */
  public Field2d getField() {
    return field;
  }

  /**
   * get the index of the Translation2d array of rotation points for evasive maneuvers
   * @return index
   */
  public int getRotationPointIdx() {
    return rotationPoint;
  }

  /**
   * set the index of the Translation2d array of rotation points for evasive maneuvers
   * @param idx
   */
  public void setRotationPointIdx(int idx) {
    rotationPoint = idx;
  }

  /**
   * get the trajectory config of the robot for autonomous
   * @return TrajaetoryConfig trajectoryConfig
   */
  public TrajectoryConfig getTrajectoryConfig() {
    return trajectoryConfig;
  }

  /**
   * set state as autonomous
   */
  public void setAutonomous() {
    state = DrivetrainState.AUTON_PATH;
  }

  /**
   * set state as joystick drive
   */
  public void setJoystick() {
    state = DrivetrainState.JOYSTICK_DRIVE;
  }

  /**
   * set state as disabled
   */
  public void setDisabled() {
    state = DrivetrainState.DISABLED;
  }

  /**
   * Reset the modules encoders
   */
  public void resetModules() {
    for (SwerveModule m:motors) {
      m.resetEncoders();
    }
  }

  /**
   * set the modes
   * @param mode the mode
   */
  public void setModes(NeutralMode mode){
    for (SwerveModule m:motors) {
      m.setMode(mode);
    }
  }

  /**
   * Toggle drivetrain field oriented mode
   */
  public void toggleMode() {
    this.fieldOriented = !this.fieldOriented;
  }
}