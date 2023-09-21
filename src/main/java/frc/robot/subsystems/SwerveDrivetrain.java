// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. Aman :)

package frc.robot.subsystems;

import java.sql.Time;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TestChassisSpeeds;
import frc.robot.commands.TestSwerveModule;


public class SwerveDrivetrain extends SubsystemBase {
  // state stuff
  public static enum DrivetrainState {
    AUTON_PATH, JOYSTICK_DRIVE, DISABLED
  }

  private DrivetrainState state;

  // kinematics stuff
  private SwerveDriveKinematics swerveKinematics;
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModule[] modules;
  private int rotationPoint = 0;
  public boolean fieldOriented = true;

  // Auton Stuff
  private SwerveDriveOdometry odometry;
  private Field2d field;
  public PIDController xController;
  public PIDController yController;
  public ProfiledPIDController thetaController;
  public PIDController rotationControllerFeedBack;
  private TrajectoryConfig trajectoryConfig;
  // sensors
  private AHRS gyro;
  // private PigeonIMU gyro;
  private double gyroOffset_deg = 0; // degrees

  private Logger logger;
  
  private DriveSimulationData driveSimData;
    
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
      Constants.SwerveDrivetrain.m_frontRightLocation, 
      Constants.SwerveDrivetrain.m_frontLeftLocation, 
      Constants.SwerveDrivetrain.m_backLeftLocation, 
      Constants.SwerveDrivetrain.m_backRightLocation);
    
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    modules = new SwerveModule[4];

    modules[0] = new SwerveModule(
      "FrontRight",
      Constants.SwerveDrivetrain.m_frontRightDriveID,
      Constants.SwerveDrivetrain.m_frontRightTurnID,
      Constants.SwerveDrivetrain.m_frontRightEncoderID,
      false,
      true,
      false,
      Constants.SwerveDrivetrain.m_frontRightEncoderOffset,
      modulePositions[0]);

    modules[1] = new SwerveModule(
      "FrontLeft",
      Constants.SwerveDrivetrain.m_frontLeftDriveID,
      Constants.SwerveDrivetrain.m_frontLeftTurnID,
      Constants.SwerveDrivetrain.m_frontLeftEncoderID,
      false,
      true,
      false,
      Constants.SwerveDrivetrain.m_frontLeftEncoderOffset,
      modulePositions[1]);

    modules[2] = new SwerveModule(
      "BackLeft",
      Constants.SwerveDrivetrain.m_backLeftDriveID,
      Constants.SwerveDrivetrain.m_backLeftTurnID,
      Constants.SwerveDrivetrain.m_backLeftEncoderID,
      false,
      true,
      false,
      Constants.SwerveDrivetrain.m_backLeftEncoderOffset,
      modulePositions[2]);

    modules[3] = new SwerveModule(
      "BackRight",
      Constants.SwerveDrivetrain.m_backRightDriveID,
      Constants.SwerveDrivetrain.m_backRightTurnID,
      Constants.SwerveDrivetrain.m_backRightEncoderID,
      false,
      true,
      false,
      Constants.SwerveDrivetrain.m_backRightEncoderOffset,
      modulePositions[3]);

    field = new Field2d();

    xController = new PIDController(Constants.SwerveDrivetrain.m_x_control_P, Constants.SwerveDrivetrain.m_x_control_I, Constants.SwerveDrivetrain.m_x_control_D);
    yController = new PIDController(Constants.SwerveDrivetrain.m_y_control_P, Constants.SwerveDrivetrain.m_y_control_I, Constants.SwerveDrivetrain.m_y_control_D);
    thetaController = new ProfiledPIDController(
      Constants.SwerveDrivetrain.m_r_control_P,
      Constants.SwerveDrivetrain.m_r_control_I,
      Constants.SwerveDrivetrain.m_r_control_D,
      Constants.SwerveDrivetrain.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    rotationControllerFeedBack = new PIDController(
      Constants.SwerveDrivetrain.m_r_control_P,
      Constants.SwerveDrivetrain.m_r_control_I,
      Constants.SwerveDrivetrain.m_r_control_D);

    trajectoryConfig = new TrajectoryConfig(
      Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, 
      Constants.SwerveDrivetrain.kMaxAutonDriveAcceleration);
    trajectoryConfig.setKinematics(swerveKinematics);
    
    state = DrivetrainState.JOYSTICK_DRIVE; 

    driveSimData = new DriveSimulationData(new SwerveDriveOdometry(swerveKinematics, new Rotation2d(), modulePositions), field);
    odometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d(), modulePositions);
  }

  public SwerveModulePosition[] getModulePositions(){
    return modulePositions;
  }
  
  /**
   * drive robo forward
   * @param mps velocity
   */
  public void driveForward(double mps){
    setSpeeds(mps, 0, 0, Constants.SwerveDrivetrain.rotatePoints[0]);
  }

  /**
   * Zero the physical gyro
   */
  public void zeroHeading() {
    gyro.reset();
  }

  public void setGyroOffset_deg(double offset_deg){
    this.gyroOffset_deg = offset_deg;
  }

  /**
   * <h3>get the heading of the physical gyro </h3> <br></br>
   * use IEEEremainder because it uses formula:
   * dividend - (divisor x Math.Round(dividend / divisor))
   * versus the remainder operator (%) which uses:
   * (Math.Abs(dividend) - (Math.Abs(divisor) x (Math.Floor(Math.Abs(dividend) / Math.Abs(divisor))))) x Math.Sign(dividend)
   * 
   * @return heading angle in degrees
   */
  public double getHeading() {
    if (Constants.DataLogging.currMode == Constants.DataLogging.Mode.SIM) {
      return Math.IEEEremainder(Math.toDegrees(driveSimData.getHeading()), 360.0);
    }
    return -Math.IEEEremainder(gyro.getYaw() - gyroOffset_deg, 360.0);
  }

  /**
   * get the relative heading of the robot based off of the chassis speeds
   * @return relative angle in degrees
   */
  public double getRelativeHeading() {
    SwerveModuleState[] states = getOutputModuleStates();

    // logger.recordOutput("Current SwerveDriveState [LF, LB, RF, RB]", states);

    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(states);
    double angle = Math.atan(speeds.vyMetersPerSecond/speeds.vxMetersPerSecond);
    return Math.toDegrees(angle);
  }

  /**
   * Gets the angle the robot is tilted
   *
   * @return the pitch degree
   */
  public double getPitchAngle(){
    return gyro.getPitch();
  }

  /**
   * gets the current gyro yaw value
   * @return yaw in degrees
   */
  public double getYaw(){
    return gyro.getYaw();
  }

  /**
   * gets the current gyro roll value
   * @return roll in degrees
   */
  public double getRoll(){
    return gyro.getRoll();
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
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Robot Heading", getHeading()); //i don't think we need to know this
    SmartDashboard.putData("Field", field);
    for (SwerveModule m : modules) {
       m.logMeasuredData();
    }

    SmartDashboard.putNumber("pitch", getPitchAngle());
    SmartDashboard.putNumber("rotation", gyro.getRotation2d().getDegrees());

    // logger.recordOutput("NavXHeadingRad", getRotation2d().getRadians());
    // logger.recordOutput("NavXHeadingDeg", getRotation2d().getDegrees());
    // logger.recordOutput("OdometryHeadingRad", getPose().getRotation().getRadians());
    // logger.recordOutput("OdometryHeadingDeg", getPose().getRotation().getDegrees());

    odometry.update(getRotation2d(), modulePositions);

    if(Constants.DataLogging.currMode != Constants.DataLogging.Mode.SIM){
      field.setRobotPose(
        odometry.getPoseMeters().getX(),
        odometry.getPoseMeters().getY(),
        getRotation2d()
      );
    }

    logger.recordOutput("Robot Location", getPose());
    logger.recordOutput("Robot Pose X", getPose().getX());
    logger.recordOutput("Robot Pose Y", getPose().getY());
    logger.recordOutput("Robot Location W deg", getPose().getRotation().getDegrees());
    logger.recordOutput("TrueSwerveDrivetrainModuleStates", getOutputModuleStates());
  }

  public void simulationPeriodic() {
    SmartDashboard.putNumber("Chassis Turn Speed", swerveKinematics.toChassisSpeeds(getOutputModuleStates()).omegaRadiansPerSecond);
    driveSimData.quadrature(swerveKinematics.toChassisSpeeds(getOutputModuleStates()).omegaRadiansPerSecond, modulePositions);
  }

  /**
   * stop the swerve modules
   */
  public void stopModules() {
    for (SwerveModule m : modules) {
      m.disableModule();
    }
  }

  /**
   * show commands of the tests on each of the modules
   */
  public void setupTests(){
    for(SwerveModule m: modules) {
      SmartDashboard.putData(m.getSwerveID() + "/RunTurnTest", new TestSwerveModule(this, m));
    }
    SmartDashboard.putData("RunChassisTests", new TestChassisSpeeds(this));
  }

  /**
   * get the swerve module
   * @param moduleID
   * @return SwerveModule [0 = FR, 1 = FL, 2 = BL, 3 = BR]
   */
  public SwerveModule getModule(int moduleID) {
    return modules[moduleID];
  }

  /**
   * set the states of all four modules
   * @param states
   */
  public void setModuleStates(SwerveModuleState[] states) {
    Logger.getInstance().recordOutput("SwerveModuleStatesDesired", states);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrivetrain.kMaxSpeedMPS);
    Logger.getInstance().recordOutput("SwerveModuleStatesDesiredNorm", states);
    for (int i = 0; i < modules.length; i++)
    {
      modules[i].setDesiredState(states[i]);
    }
    // TODO add a log of the array of post optimized and pre-optimized states
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
    setModuleStates(moduleStates);
  }

  /**
   * Set the speeds in field oriented (v_forward is the x direction, side is the y direction)
   * @param v_forwardMps
   * @param v_sideMps
   * @param v_rot pos is counterclockwise, neg is clockwise
   */
  public void setSpeedsFieldOriented(double v_forwardMps, double v_sideMps, double v_rot) {
    if(DriverStation.getAlliance() == Alliance.Red) {
      v_forwardMps = -v_forwardMps;
      v_sideMps = -v_sideMps;
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(v_forwardMps, v_sideMps, v_rot, getPose().getRotation());
    SmartDashboard.putString("ChassisSpeedsFO", speeds.toString());
    SwerveModuleState[] moduleStates = this.swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
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
        getOutputModuleStates()
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
        getOutputModuleStates()
    );
    return speeds.omegaRadiansPerSecond;
  }

  /**
   * get the linear speed of the robot in mps
   * @return linear speed
   */
  public double getTranslationSpeedMPS() {
    Translation2d linearVel = getLinearVelocity();
    return Math.hypot(linearVel.getX(), linearVel.getY());
  }
  
  /**
   * get the actual states of all swerve modules
   * @return SwerveModuleState[] states
   */
  public SwerveModuleState[] getOutputModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++)
    {
      states[i] = modules[i].getState();
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
    SmartDashboard.putBoolean("Reset Odometry", true);
    SmartDashboard.putNumber("Reset angle", getRotation2d().getDegrees());
    odometry.resetPosition(getRotation2d(), modulePositions, pose);
    //SmartDashboard.putNumber("Reset angle", get)
    if (Constants.DataLogging.currMode == Constants.DataLogging.Mode.SIM) {
      driveSimData.resetOdometry(getRotation2d(), modulePositions, pose);
    }
    SmartDashboard.putBoolean("Reset Odometry", false);
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
    for (SwerveModule m:modules) {
      m.resetEncoders();
    }
  }

  public void resetModuleDrive() {
    for (SwerveModule m:modules) {
      m.resetDriveEncoders();
    }
  }

  /**
   * set the modes
   * @param mode the mode
   */
  public void setModes(NeutralMode mode){
    for (SwerveModule m:modules) {
      m.setMode(mode);
    }
    SmartDashboard.putBoolean("Brake Mode", (mode == NeutralMode.Brake));
    logger.recordOutput("Brake Mode", (mode == NeutralMode.Brake));
  }

  /**
   * Toggle drivetrain field oriented mode
   */
  public void toggleMode() {
    this.fieldOriented = !this.fieldOriented;
  }

  /**
   * uses PID to try and hold the current heading of the robot
   * @param heading
   */
  public double holdHeading(Rotation2d heading) {
    double v_w = Constants.JoystickControls.kPJoystick * (heading.getRadians() - getRotation2d().getRadians()); //thetaController.calculate(getRotation2d().getRadians());
    // this.setSpeeds(0, 0, v_w, Constants.SwerveDrivetrain.rotatePoints[this.getRotationPointIdx()]);
    return v_w;
  }

  public PPSwerveControllerCommand getAutonPathCommand(PathPlannerTrajectory trajectory) {
    return new PPSwerveControllerCommand(
      trajectory, 
      this::getPose,
      this.swerveKinematics,
      this.xController, 
      this.yController,
      this.rotationControllerFeedBack,
      this::setModuleStates,
      false,
      this
    );
  }

  public SwerveAutoBuilder getAutonBuilder(HashMap<String, Command> eventMap) {
    return new SwerveAutoBuilder(
      this::getPose,
      this::resetFakeOdometry,
      this.swerveKinematics, 
      new PIDConstants(Constants.SwerveDrivetrain.m_x_control_P, Constants.SwerveDrivetrain.m_x_control_I, Constants.SwerveDrivetrain.m_x_control_D),
      new PIDConstants(Constants.SwerveDrivetrain.m_r_control_P, Constants.SwerveDrivetrain.m_r_control_I, Constants.SwerveDrivetrain.m_r_control_D), 
      this::setModuleStates,
      eventMap, 
      false,
      this);
  }

  private void resetFakeOdometry(Pose2d pose) {
    return;
  }

public DrivetrainState getState() {
    return state;
}
}
