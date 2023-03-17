// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// licene deez nuts

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Map;

import org.photonvision.PhotonCamera;
import frc.robot.utils.PhotonPoseEstimator;
import frc.robot.utils.PhotonPoseEstimator.PoseStrategy;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain.DrivetrainState;

public class Localization extends SubsystemBase {
  private PhotonCamera camera1;
  private PhotonCamera camera2;
  private PhotonPoseEstimator camera1Estimator;
  private PhotonPoseEstimator camera2Estimator;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDrivetrain swerveDrivetrain;
  private AprilTagFieldLayout fieldLayout;
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(5));
  private final Field2d field;
  private Logger logger = Logger.getInstance();
  
  private boolean aligning;

  private boolean firstSee;

  public Localization(SwerveDrivetrain swerveDrivetrain) {
    //this.camera1 = new PhotonCamera(Constants.VisionConstants.kCamera1Name);
    this.camera2 = new PhotonCamera(Constants.VisionConstants.kCamera2Name);
    this.swerveDrivetrain = swerveDrivetrain;
    this.field = swerveDrivetrain.getField();
    
    try {
      this.fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch(IOException e) {
      System.err.println("[Localization constructor] Error loading from resource");
    }
    /*camera1Estimator = new PhotonPoseEstimator(
      fieldLayout,
      PoseStrategy.MULTI_TAG_PNP,
      camera1,
      Constants.VisionConstants.cam1ToRobot);*/
    //camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // Rotation2d correctedAngle = Rotation2d.fromDegrees(swerveDrivetrain.getPose().getRotation().getDegrees() + 180)

    camera2Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera2, Constants.VisionConstants.cam2ToRobot);
    camera2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator = new SwerveDrivePoseEstimator(swerveDrivetrain.getKinematics(), 
      normalizeAngle(swerveDrivetrain.getPose().getRotation()), 
      // correctedAngle,
      swerveDrivetrain.getModulePositions(), 
      new Pose2d(), stateStdDevs, visionMeasurementStdDevs); //Replace this with the starting pose in auton

    for(int i = 1;i<=8;i++) {
    //   SmartDashboard.putString("WPILIB Apriltag"+i, fieldLayout.getTagPose(i).get().toString());
      //  logger.recordOutput("Apriltag " + i + " Location", Constants.VisionConstants.aprilTags.get(i));
    }  
  }

  @Override
  public void periodic() {
    //camera1Estimator.setReferenceTheta(swerveDrivetrain.getPose().getRotation().getRadians());
    //camera2Estimator.setReferenceTheta(swerveDrivetrain.getPose().getRotation().getRadians());
    //Optional<EstimatedRobotPose> result1 = camera1Estimator.update();
    Optional<EstimatedRobotPose> result2 = camera2Estimator.update();
    Pose2d result = combinePoses(null, result2);

    if(!firstSee && swerveDrivetrain.getState() != DrivetrainState.AUTON_PATH && result != null) {
      Pose2d curr = getCurrentPose();
      if(curr!=null){
        swerveDrivetrain.resetOdometry(curr);
      }

      firstSee = true;
    }

    SmartDashboard.putBoolean("chicken robot present", result != null);
    if(result != null) {
      //update here rotation to whatever gyro gives us
      result = new Pose2d(result.getX(), result.getY(), normalizeAngle(swerveDrivetrain.getPose().getRotation()));
      SmartDashboard.putString("chicken robot pose", result.toString());

      //If aligning, reset pose to whatever camera gives us
      if(aligning){
        resetPoseEstimator(result);
      }
      else{
        poseEstimator.addVisionMeasurement(result, Timer.getFPGATimestamp());
        SmartDashboard.putNumber("chicken Timer", Timer.getFPGATimestamp());
      }
    }
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), normalizeAngle(swerveDrivetrain.getPose().getRotation()), swerveDrivetrain.getModulePositions());
    Pose2d currPose = getCurrentPose();
    if(currPose != null){
      field.setRobotPose(currPose); 
    }
    log();

    Pose2d robotPose = getCurrentPose();
    
    if(robotPose==null) return;

    Pose2d poseToGoTo = getClosestScoringLoc();
    SmartDashboard.putString("chicken robot pose", getCurrentPose().toString());
    SmartDashboard.putNumber("chicken robot gyro rot", normalizeAngle(swerveDrivetrain.getPose().getRotation()).getDegrees());
    SmartDashboard.putNumber("chicken robot score theta",  (poseToGoTo.getRotation().getDegrees()));
    SmartDashboard.putNumber("chicken robot error theta", poseToGoTo.getRotation().getDegrees() -  computeThetaError(normalizeAngle(swerveDrivetrain.getPose().getRotation()).getDegrees(), false));
    debugPID();
    SmartDashboard.putString("chicken - closest loc", poseToGoTo.toString());

    logger.recordOutput("chicken Robot Location", getCurrentPose());
    // logger.recordOutput("chicken Robot Pose X", getCurrentPose().getX());
    // logger.recordOutput("chicken Robot Pose Y", getCurrentPose().getY());
    //logger.recordOutput("chicken Robot Location W deg", getCurrentPose().getRotation().getDegrees());
    //logger.recordOutput("chicken Robot Location W rad", getCurrentPose().getRotation().getRadians());
  }

  /**
   * Initializes pose estimator and configures stdevs
   * @param pose
   */
  public void resetPoseEstimator(Pose2d pose){
    // Rotation2d correctedAngle = Rotation2d.fromDegrees(swerveDrivetrain.getPose().getRotation().getDegrees() + 180);
    poseEstimator.resetPosition(normalizeAngle(swerveDrivetrain.getPose().getRotation()), swerveDrivetrain.getModulePositions(), pose);
  }

  public void resetCameraEstimators(){
    //camera1Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera1, Constants.VisionConstants.cam1ToRobot);
    //camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    camera2Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera2, Constants.VisionConstants.cam2ToRobot);
    camera2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }
  
  /**
   * @return current pose according to pose estimator
   */
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @param in the initial rotation of the gyroscope
   * @return the field relative rotation of the bot
   */
  public Rotation2d normalizeAngle(Rotation2d in){
    double curr = in.getRadians();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      return new Rotation2d(Math.signum(curr)*(Math.abs(curr)-Math.PI));
    }
    return in;
  }

  /**
   * @param fieldRelative FIELD RELATIVE value of gyroscope
   * @param radians radians or not
   * @return the theta value to subtractto subtract to get error in theta
   */
  public double computeThetaError(double fieldRelative, boolean radians){
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue && fieldRelative<0){
      fieldRelative += radians ? 2*Math.PI : 360;
    }
    return fieldRelative;
  }

  public Pose2d combinePoses(Optional<EstimatedRobotPose> result1, Optional<EstimatedRobotPose> result2) {
    Pose3d first;
    Pose3d second;
    if(result1==null){
      if(result2.isPresent()){
        second = result2.get().estimatedPose;
        SmartDashboard.putString("chicken cam2 pose", second.toString());
  
        return second.toPose2d();
      }
      return null;
    }
    if(result1.isPresent() && result2.isPresent()){
      first = result1.get().estimatedPose;
      second = result2.get().estimatedPose;
      SmartDashboard.putString("chicken cam1 pose", first.toString());
      SmartDashboard.putString("chicken cam2 pose", second.toString());
      return new Pose2d((first.getX()+second.getX())/2, (first.getY()+second.getY())/2, new Rotation2d((first.getRotation().getZ()+second.getRotation().getZ())/2));
    }
    if(result1.isPresent()){
      first = result1.get().estimatedPose;
      SmartDashboard.putString("chicken cam1 pose", first.toString());
      return first.toPose2d();
    }
    if(result2.isPresent()){
      second = result2.get().estimatedPose;
      SmartDashboard.putString("chicken cam2 pose", second.toString());

      return second.toPose2d();
    }
    return null;
  }
  
  /**
   * Get scoring loc to left of closest
   * @return pose2d of left to closest score loc
   */
  public Pose2d getLeftScoreLoc(){
    Pose2d orig = getClosestScoringLoc();

    Map<Integer, Pose2d> scoreCols = Constants.VisionConstants.kBlueScoreCols;
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
      scoreCols = Constants.VisionConstants.kRedScoreCols;
    } 

    //Loop through cols
    for(int i : scoreCols.keySet()) {
      Pose2d pose = scoreCols.get(i);
     
      if (pose.equals(orig)){
        if (i+1 < scoreCols.size()){
          return scoreCols.get(i+1);
        }
      }
    }
      
    return null;
  }

  /**
   * Get scoring loc to right of closest
   * @return pose2d of left to closest score loc
   */
  public Pose2d getRightScoreLoc(){
    Pose2d orig = getClosestScoringLoc();

    Map<Integer, Pose2d> scoreCols = Constants.VisionConstants.kBlueScoreCols;
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
      scoreCols = Constants.VisionConstants.kRedScoreCols;
    } 

    //Loop through cols
    for(int i : scoreCols.keySet()) {
      Pose2d pose = scoreCols.get(i);
     
      if (pose.equals(orig)){
        if (i-1 >= 0){
          return scoreCols.get(i-1);
        }
      }
    }
      
    return null;
  }

  /**
   * Gets the closest scoring location using SwerveDrivePoseEstimator
   * @return Returns the Pose2d of the scoring col
   */
  public Pose2d getClosestScoringLoc() {
    Map<Integer, Pose2d> scoreCols = Constants.VisionConstants.kBlueScoreCols;
    
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
      scoreCols = Constants.VisionConstants.kRedScoreCols;
    } 

    Pose2d minCol = null;
    double minDist = Double.MAX_VALUE;

    //Loop through cols
    for(int i : scoreCols.keySet()) {
      Pose2d pose = scoreCols.get(i);
      double dy = Math.abs(poseEstimator.getEstimatedPosition().getY() - pose.getY());

      //Shortest dist away
      if(dy < minDist) {
        minDist = dy;
        minCol = pose;
      }
    }
    //SmartDashboard.putNumber("chicken scoring loc", num);
    return minCol;
  }

  /**
   * Teels whether or not certain score loc is for cone
   * @param scorecol the column
   * @return true if for cones false if for cubes
   */
  public boolean isConeScoreLoc(int scoreCol){
    if (scoreCol == 1 || scoreCol == 3 || scoreCol == 4 || 
        scoreCol == 6 || scoreCol == 7 || scoreCol == 9){
        return true;  
    }
    return false;
  }

   /**
   * @param relLoc the target relative to camera which is (0, 0)
   * @return distance from target
   */
  public static double distFromTag(Transform3d relLoc) {
    return Math.sqrt(relLoc.getX() * relLoc.getX() + 
        relLoc.getY() * relLoc.getY() + 
        relLoc.getZ() * relLoc.getZ());
  }

  /**
   * @param initialPose the initial pose
   * @param finalPose the final pose
   * @return the distance between the two poses
   */
  public static double distFromTag(Pose2d initialPose, Pose2d finalPose){
    return Math.sqrt(Math.pow(initialPose.getX()-finalPose.getX(),2)+Math.pow(initialPose.getY()-finalPose.getY(),2));
  }

  /**
   * Returns whether the robot is currently aligning or not
   * @return whether or not the robot is currently aligning
   */
  public boolean isAligning(){
    return aligning;
  }

  /**
   * Sets whether or not the robot is aligning
   * @param inc whether the bot is aligning
   */
  public void setAligning(boolean inc){
    aligning = inc;
  }

  /**
   * Log stuff
   */
  public void log() {
    var pos = getCurrentPose();
    if(pos != null){
      SmartDashboard.putString("chicken logged Estimated Position", pos.toString());
    }
    
    //SmartDashboard.putBoolean("chicken cam 1 can see", camera1.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("chicken cam 2 can see", camera2.getLatestResult().hasTargets());
  }

  /**
   * Log stuff for debugging pid
   */
  public void debugPID(){
    Pose2d robotPose = getCurrentPose();
    if(robotPose==null){
      SmartDashboard.putBoolean("chicken pose is null", true);
      return;
    }

    // SmartDashboard
    Pose2d poseToGoTo = getClosestScoringLoc();
    //Rotation2d realTheta = normalizeAngle(swerveDrivetrain.getPose().getRotation());
    SmartDashboard.putNumber("chicken raw theta", swerveDrivetrain.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("chicken robo theta", robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("chicken score theta",  (poseToGoTo.getRotation().getDegrees()));
    SmartDashboard.putNumber("chicken error theta", poseToGoTo.getRotation().getDegrees() - computeThetaError(robotPose.getRotation().getDegrees(), false));
    SmartDashboard.putNumber("chicken scoring x", poseToGoTo.getX());
    SmartDashboard.putNumber("chicken scoring y", poseToGoTo.getY());
    SmartDashboard.putNumber("chicken robo x", robotPose.getX());
    SmartDashboard.putNumber("chicken robo y", robotPose.getY());
    SmartDashboard.putNumber("chicken distance from final", Localization.distFromTag(robotPose, poseToGoTo));
    SmartDashboard.putNumber("chicken error x", poseToGoTo.getX() - robotPose.getX());
    SmartDashboard.putNumber("chicken error y", poseToGoTo.getY() - robotPose.getY());
  }
}