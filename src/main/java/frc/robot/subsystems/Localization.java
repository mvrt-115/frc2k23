// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// licene deez nuts

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Localization extends SubsystemBase {
  private PhotonCamera camera1;
  private PhotonCamera camera2;
  private PhotonPoseEstimator camera1Estimator;
  private PhotonPoseEstimator camera2Estimator;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDrivetrain swerveDrivetrain;
  private AprilTagFieldLayout fieldLayout;
  private final Field2d field;
  private boolean aligning;

  public Localization(SwerveDrivetrain swerveDrivetrain) {
    this.camera1 = new PhotonCamera(Constants.VisionConstants.kCamera1Name);
    this.camera2 = new PhotonCamera(Constants.VisionConstants.kCamera2Name);
    this.swerveDrivetrain = swerveDrivetrain;
    this.field = swerveDrivetrain.getField();
    try {
      this.fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch(IOException e) {
      System.err.println("[Localization 2 constructor] Error loading from resource");
    }
    poseEstimator = new SwerveDrivePoseEstimator(swerveDrivetrain.getKinematics(), 
      swerveDrivetrain.getRotation2d(), 
      swerveDrivetrain.getModulePositions(), 
      new Pose2d()); //Replace this with the starting pose in auton    
    camera1Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera1, Constants.VisionConstants.cam1ToRobot);
    camera2Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera2, Constants.VisionConstants.cam2ToRobot);
   // resetPoseEstimator(new Pose2d());
    for(int i = 1;i<=8;i++){
      SmartDashboard.putString("WPILIB Apriltag"+i, fieldLayout.getTagPose(i).get().toString());
      SmartDashboard.putString("Apriltag"+i, Constants.VisionConstants.aprilTags.get(i).toString());
    }  
  }

  @Override
  public void periodic() {
    Pose2d result = combinePoses();

    SmartDashboard.putBoolean("robot present", result != null);
    //Pose2d camPose = camera1Estimator.update().get().estimatedPose.toPose2d();
    if(result != null) {
      //update here rotation to whatever gyro gives us
      result = new Pose2d(result.getX(), result.getY(), swerveDrivetrain.getRotation2d());
      SmartDashboard.putString("robot pose", result.toString());

      //If aligning, reset pose to whatever camera gives us
      if(aligning){
        resetPoseEstimator(result);
      }
      else{
        double latency = 0;
        PhotonPipelineResult cam1Result = camera1.getLatestResult();
        PhotonPipelineResult cam2Result = camera2.getLatestResult();

        if(cam1Result.hasTargets()){
          latency = cam1Result.getLatencyMillis();

        } if(cam2Result.hasTargets()){
            if(latency==0){
              latency = cam2Result.getLatencyMillis();
            }
            else{
              latency+=cam2Result.getLatencyMillis();
              latency/=2;
            }
        }
        poseEstimator.addVisionMeasurement(result, latency);
      }
    }
    Pose2d currPose = getCurrentPose();
    if(currPose != null){
      field.setRobotPose(currPose); 
    }

    //poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerveDrivetrain.getRotation2d(), swerveDrivetrain.getModulePositions());
    field.setRobotPose(getCurrentPose());
    log();

    Pose2d robotPose = getCurrentPose();
    if(robotPose==null) return;

    Pose2d poseToGoTo = Constants.VisionConstants.kRedScoreCols.get(5);
    SmartDashboard.putString("robot pose", getCurrentPose().toString());
    SmartDashboard.putNumber("robot gyro rot", swerveDrivetrain.getRotation2d().getDegrees());
    SmartDashboard.putNumber("robot score theta",  (poseToGoTo.getRotation().getDegrees()));
    SmartDashboard.putNumber("robot error theta", (poseToGoTo.getRotation().getDegrees()) - swerveDrivetrain.getRotation2d().getDegrees());
    debugPID();
  }

  /**
   * Initializes pose estimator and configures stdevs
   * @param pose
   */
  public void resetPoseEstimator(Pose2d pose){
    poseEstimator.resetPosition(swerveDrivetrain.getRotation2d(), swerveDrivetrain.getModulePositions(), pose);
  }

  public void resetTemp(){
    camera1Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera1, Constants.VisionConstants.cam1ToRobot);
  }
  /**
   * @return current pose according to pose estimator
   */
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d combinePoses() {
    Optional<EstimatedRobotPose> result1 = camera1Estimator.update();
    Optional<EstimatedRobotPose> result2 = camera2Estimator.update();

    
    Pose3d first;
    Pose3d second;
    if(result1.isPresent() && result2.isPresent()){
      first = result1.get().estimatedPose;
      second = result2.get().estimatedPose;
      SmartDashboard.putString("cam1 pose", first.toString());
      SmartDashboard.putString("cam2 pose", second.toString());
      return new Pose2d((first.getX()+second.getX())/2, (first.getY()+second.getY())/2, new Rotation2d((first.getRotation().getZ()+second.getRotation().getZ())/2));
    }
    if(result1.isPresent()){
      first = result1.get().estimatedPose;
      SmartDashboard.putString("cam1 pose", first.toString());
      return first.toPose2d();
    }
    if(result2.isPresent()){
      second = result2.get().estimatedPose;
      SmartDashboard.putString("cam2 pose", second.toString());

      return second.toPose2d();
    }
    return null;
  }
  /**
  * Weights multiple targets using distances (linear)
  * @return the weighted Pose2d
  */
  private Pose2d weightTargets() {
    SmartDashboard.putBoolean("robot field oriented", swerveDrivetrain.fieldOriented);
    Pose2d cam1Pose = null;
    Pose2d cam2Pose = null;
    PhotonPipelineResult cam1Result = camera1.getLatestResult();
    PhotonPipelineResult cam2Result = camera2.getLatestResult();
    if(cam1Result.hasTargets()){
      //log raw value for debugging
      SmartDashboard.putString("Cam1 Raw", cam1Result.getBestTarget().getBestCameraToTarget().toString());
      cam1Pose = weightMultiTargets((ArrayList<PhotonTrackedTarget>) cam1Result.targets, Constants.VisionConstants.cam1ToRobot);
      SmartDashboard.putString("Cam1 FieldRelative", cam1Pose.toString());
    }
    if(cam2Result.hasTargets()){
      //log raw value for debugging
      SmartDashboard.putString("Cam2 Raw", cam2Result.getBestTarget().getBestCameraToTarget().toString());
      cam2Pose = weightMultiTargets((ArrayList<PhotonTrackedTarget>) cam2Result.targets, Constants.VisionConstants.cam2ToRobot);
      SmartDashboard.putString("Cam2 FieldRelative", cam2Pose.toString());
    }
    //Handle cases where only one camera can see
    if(cam1Pose == null && cam2Pose==null) return null;
    if(cam1Pose == null) return cam2Pose;
    if(cam2Pose == null) return cam1Pose;
    
    double xAvg = (cam1Pose.getX() + cam2Pose.getX()) / 2;
    double yAvg = (cam1Pose.getY() + cam2Pose.getY()) / 2;
    
    //For debugging theta
    SmartDashboard.putNumber("Cam1 Theta Raw", cam1Result.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
    SmartDashboard.putNumber("Cam2 Theta Raw", cam2Result.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
    SmartDashboard.putNumber("Cam1 Theta FieldRelative", cam1Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Cam2 Theta Field Relative", cam2Pose.getRotation().getDegrees());

    //just use gyro angle
    return new Pose2d(xAvg, yAvg, swerveDrivetrain.getRotation2d());
  }

  /**
   * 
   * @param targets list of apriltag targets
   * @param camPose camera position
   * @return new camera position after weighting all targets
   */
  private Pose2d weightMultiTargets(ArrayList<PhotonTrackedTarget> targets, Transform3d camPose) {
    double[] weights = new double[targets.size()];

    //reciprocal of all the distances
    double totalSum = 0;
    Transform3d[] transforms = new Transform3d[targets.size()];
    
    for(int i = 0; i < transforms.length; i++) {
      Transform3d relLoc = targets.get(i).getBestCameraToTarget();
      transforms[i] = relLoc;
      weights[i] = 1/relLoc.getTranslation().getNorm();
      totalSum += weights[i];
    }
    
    double weightedX = 0;
    double weightedY = 0;
    double weightedRot = 0;
    
    for(int i = 0; i < targets.size(); i++) {
      Pose3d tag = Constants.VisionConstants.aprilTags.get(2);
      Pose3d robotPose = ComputerVisionUtil.objectToRobotPose(tag, transforms[i], camPose);
      weightedX += weights[i] * robotPose.getX();
      weightedY += weights[i] * robotPose.getY();
      weightedRot += weights[i] * swerveDrivetrain.getRotation2d().getRadians();
    }

    weightedX /= totalSum;
    weightedY /= totalSum;
    weightedRot /= totalSum;

    return new Pose2d(weightedX, weightedY, new Rotation2d(weightedRot));
  }

  /**
   * Gets the closest scoring location using SwerveDrivePoseEstimator
   * @return Returns the Pose2d of the scoring col
   */
  public Pose2d getClosestScoringLoc() {
    //Set the score cols depending on if blue/red
    Map<Integer, Pose2d> scoreCols = Constants.VisionConstants.kRedScoreCols;
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
    return scoreCols.get(5);
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
      SmartDashboard.putString("Logged Estimated Position", pos.toString());
    }
    
    SmartDashboard.putBoolean("Cam 1 can see", camera1.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("Cam 2 can see", camera2.getLatestResult().hasTargets());
   // Logger.getInstance().recordOutput("Robo X", lastPose.getX());
    //Logger.getInstance().recordOutput("Robo Y", lastPose.getY());
  }

  /**
   * Log stuff for debugging pid
   */
  public void debugPID(){
    Pose2d robotPose = getCurrentPose();
    if(robotPose==null){
      SmartDashboard.putBoolean("pose is null", true);
      return;
    }

    // SmartDashboard
    Pose2d poseToGoTo = Constants.VisionConstants.kRedScoreCols.get(5);

    SmartDashboard.putNumber("robo theta", (swerveDrivetrain.getRotation2d().getDegrees()));
    SmartDashboard.putNumber("score theta",  (poseToGoTo.getRotation().getDegrees()));
    SmartDashboard.putNumber("error theta", (poseToGoTo.getRotation().getDegrees()) - (swerveDrivetrain.getRotation2d().getDegrees()));
    SmartDashboard.putNumber("scoring x", poseToGoTo.getX());
    SmartDashboard.putNumber("scoring y", poseToGoTo.getY());
    SmartDashboard.putNumber("robo x", robotPose.getX());
    SmartDashboard.putNumber("robo y", robotPose.getY());
    SmartDashboard.putNumber("distance from final", Localization.distFromTag(robotPose, poseToGoTo));
    SmartDashboard.putNumber("error x", poseToGoTo.getX() - robotPose.getX());
    SmartDashboard.putNumber("error y", poseToGoTo.getY() - robotPose.getY());
  }
  
}