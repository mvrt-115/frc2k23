// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// licene deez nuts

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
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

    for(int i = 1;i<=8;i++){
      SmartDashboard.putString("WPILIB Apriltag"+i, fieldLayout.getTagPose(i).get().toString());
      SmartDashboard.putString("Apriltag"+i, Constants.VisionConstants.aprilTags.get(i).toString());
    }  
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> result1 = camera1Estimator.update();
    Optional<EstimatedRobotPose> result2 = camera2Estimator.update();
    Pose2d result = combinePoses(result1, result2);

    SmartDashboard.putBoolean("robot present", result != null);
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
        if(result1.isPresent()){
          latency = result1.get().timestampSeconds;

        } if(result2.isPresent()){
            if(latency==0){
              latency = result2.get().timestampSeconds;
            }
            else{
              latency+=result2.get().timestampSeconds;
              latency/=2;
            }
        }
        poseEstimator.addVisionMeasurement(result, latency);
      }
    }
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerveDrivetrain.getRotation2d(), swerveDrivetrain.getModulePositions());
    Pose2d currPose = getCurrentPose();
    if(currPose != null){
      field.setRobotPose(currPose); 
    }
    log();

    Pose2d robotPose = getCurrentPose();
    if(robotPose==null) return;

    Pose2d poseToGoTo = getClosestScoringLoc();
    SmartDashboard.putString("chicken - robot pose", getCurrentPose().toString());
    SmartDashboard.putNumber("chicken - robot gyro rot", swerveDrivetrain.getRotation2d().getDegrees());
    SmartDashboard.putNumber("chicken - robot score theta",  (poseToGoTo.getRotation().getDegrees()));
    SmartDashboard.putNumber("chicken - robot error theta", (poseToGoTo.getRotation().getDegrees()) - swerveDrivetrain.getRotation2d().getDegrees());
    debugPID();
  }

  /**
   * Initializes pose estimator and configures stdevs
   * @param pose
   */
  public void resetPoseEstimator(Pose2d pose){
    poseEstimator.resetPosition(swerveDrivetrain.getRotation2d(), swerveDrivetrain.getModulePositions(), pose);
  }

  public void resetCameraEstimators(){
    camera1Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera1, Constants.VisionConstants.cam1ToRobot);
    camera2Estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera2, Constants.VisionConstants.cam2ToRobot);
  }
  /**
   * @return current pose according to pose estimator
   */
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d combinePoses(Optional<EstimatedRobotPose> result1, Optional<EstimatedRobotPose> result2) {
    Pose3d first;
    Pose3d second;
    if(result1.isPresent() && result2.isPresent()){
      first = result1.get().estimatedPose;
      second = result2.get().estimatedPose;
      SmartDashboard.putString("chicken - cam1 pose", first.toString());
      SmartDashboard.putString("chicken - cam2 pose", second.toString());
      return new Pose2d((first.getX()+second.getX())/2, (first.getY()+second.getY())/2, new Rotation2d((first.getRotation().getZ()+second.getRotation().getZ())/2));
    }
    if(result1.isPresent()){
      first = result1.get().estimatedPose;
      SmartDashboard.putString("chicken - cam1 pose", first.toString());
      return first.toPose2d();
    }
    if(result2.isPresent()){
      second = result2.get().estimatedPose;
      SmartDashboard.putString("chicken - cam2 pose", second.toString());

      return second.toPose2d();
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
    int num = 0;

    //Loop through cols
    for(int i : scoreCols.keySet()) {
      Pose2d pose = scoreCols.get(i);
      double dy = Math.abs(poseEstimator.getEstimatedPosition().getY() - pose.getY());

      //Shortest dist away
      if(dy < minDist) {
        minDist = dy;
        num = i;
        minCol = pose;
      }
    }
    SmartDashboard.putNumber("scoring loc", num);
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
      SmartDashboard.putString("chicken - logged Estimated Position", pos.toString());
    }
    
    SmartDashboard.putBoolean("chicken - cam 1 can see", camera1.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("chicken -cam 2 can see", camera2.getLatestResult().hasTargets());
  }

  /**
   * Log stuff for debugging pid
   */
  public void debugPID(){
    Pose2d robotPose = getCurrentPose();
    if(robotPose==null){
      SmartDashboard.putBoolean("chicken - pose is null", true);
      return;
    }

    // SmartDashboard
    Pose2d poseToGoTo = getClosestScoringLoc();

    SmartDashboard.putNumber("chicken - robo theta", (swerveDrivetrain.getRotation2d().getDegrees()));
    SmartDashboard.putNumber("chicken - score theta",  (poseToGoTo.getRotation().getDegrees()));
    SmartDashboard.putNumber("chicken - error theta", (poseToGoTo.getRotation().getDegrees()) - (swerveDrivetrain.getRotation2d().getDegrees()));
    SmartDashboard.putNumber("chicken - scoring x", poseToGoTo.getX());
    SmartDashboard.putNumber("chicken - scoring y", poseToGoTo.getY());
    SmartDashboard.putNumber("chicken - robo x", robotPose.getX());
    SmartDashboard.putNumber("chicken - robo y", robotPose.getY());
    SmartDashboard.putNumber("chicken - distance from final", Localization.distFromTag(robotPose, poseToGoTo));
    SmartDashboard.putNumber("chicken - error x", poseToGoTo.getX() - robotPose.getX());
    SmartDashboard.putNumber("chicken - error y", poseToGoTo.getY() - robotPose.getY());
  }
}