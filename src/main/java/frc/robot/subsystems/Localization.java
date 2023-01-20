// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// licene deez nuts

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
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
  private PhotonCamera camera;
  private SwerveDrivetrain swerveDrivetrain;
  private Pose2d roboPose;
  private final SwerveDrivePoseEstimator poseEstimator;
  
  private final Field2d field;
  private DriverStation.Alliance alliance;

  public Localization(SwerveDrivetrain swerveDrivetrain, DriverStation.Alliance alliance) {
    this.camera = new PhotonCamera(Constants.VisionConstants.kCameraName);
    this.swerveDrivetrain = swerveDrivetrain;
    this.field = swerveDrivetrain.getField();
    roboPose = new Pose2d();

    //Measure this pose before initializing the class
    poseEstimator = new SwerveDrivePoseEstimator(swerveDrivetrain.getKinematics(), 
                                                  swerveDrivetrain.getRotation2d(), 
                                                  swerveDrivetrain.getModulePositions(), 
                                                  swerveDrivetrain.getPose());
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    Map<Integer, Pose3d> targetPoses = Constants.VisionConstants.aprilTags;

    if (result.hasTargets()) {
      SmartDashboard.putBoolean("Found Tag(s)", true);
      double imageCaptureTime = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000d);

      //Loop through seen tags
      for (PhotonTrackedTarget target : result.getTargets()) {
        int fiducialId = target.getFiducialId();

        if (fiducialId >= 1 && fiducialId <= targetPoses.size()) {
          Transform3d relLoc = target.getBestCameraToTarget();
          Pose3d tag = targetPoses.get(target.getFiducialId());

          Pose2d roboOnField = new Pose2d(roboPose.getX(), roboPose.getY(), roboPose.getRotation());
          field.getObject("VisionRobot" + fiducialId).setPose(roboOnField); //Robot pose according to apriltags
          poseEstimator.addVisionMeasurement(roboOnField, imageCaptureTime);
        }
      }
      log();
    }
    else{
      SmartDashboard.putBoolean("Found Tag(s)", false);
    }
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerveDrivetrain.getRotation2d(), swerveDrivetrain.getModulePositions());

    field.setRobotPose(getCurrentPose());
    roboPose = field.getRobotPose();
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Pick alignment depending on closest tag and what robot is currently holding
   * @return
   */

  /** _____
   * |_____|
   * Origin at bottom left corner of rectangle facing towards the right with CCW
   * being positive
   *
   * @return Estimated pose of robot based on closest detected AprilTag
   */
  public Pose2d getEstimatedPose() {
    PhotonPipelineResult result = camera.getLatestResult();
   
    //Best target
    PhotonTrackedTarget target = result.getBestTarget();

   
    if (target != null){
      Transform3d relLoc = target.getBestCameraToTarget();
      Pose3d tag = Constants.VisionConstants.aprilTags.get(target.getFiducialId());

      SmartDashboard.putNumber("Tag Distance", distFromTag(relLoc));
      
      return ComputerVisionUtil.objectToRobotPose(tag, relLoc, new Transform3d()).toPose2d();
    }

    return null;
   }

   /**
    * Weights multiple targets using distances (linear)
    * @param targets The result.getBestTarget() targets from each camera
    * @return Returns the weighted Pose2d
    */
   private static Pose2d weightTargets(PhotonTrackedTarget ...targets) {
      double[] weights = new double[targets.length];

      Transform3d[] transforms = new Transform3d[targets.length];
      for(int i = 0; i < transforms.length; i++) {
        Transform3d relLoc = targets[i].getBestCameraToTarget();

        transforms[i] = relLoc;
      }

      double totalDist = 0;
      for(Transform3d p : transforms) {
        totalDist += distFromTag(p);
      }

      for(int i = 0; i < weights.length; i++) {
        weights[i] = (totalDist - distFromTag(transforms[i])) / totalDist;
      }

      double weightedX = 0;
      double weightedY = 0;
      double weightedRot = 0;

      for(int i = 0; i < targets.length; i++) {
        Pose3d tag = Constants.VisionConstants.aprilTags.get(targets[i].getFiducialId());
        Pose2d robotPose = ComputerVisionUtil.objectToRobotPose(tag, transforms[i], new Transform3d()).toPose2d();
        
        weightedX += weights[i] * robotPose.getX();
        weightedY += weights[i] * robotPose.getY();
        weightedRot += weights[i] * robotPose.getRotation().getRadians();
      }

      return new Pose2d(weightedX, weightedY, new Rotation2d(weightedRot));
   }

   /**
    * Gets the best target out of those passed in
    * @param targets The tracked targets
    * @return The best target to use for localization
    */
  private PhotonTrackedTarget getBestTarget(PhotonTrackedTarget ...targets) {
      double minDist = Double.MAX_VALUE;
      PhotonTrackedTarget best = null;

      for(PhotonTrackedTarget t : targets) {
        double dist = distFromTag(t.getBestCameraToTarget());

        if(dist < minDist) {
          minDist = dist;
          best = t;
        }
      }

      return best;
  } 

  /**
   * Gets the scoring locations in range
   * @param robotPose The robot pose
   * @return Returns the Pose2d of the scoring col
   */
  public Pose2d getClosestScoringLoc(Pose2d robotPose) {
    Map<Integer, Pose2d> scoreCols = 
        alliance == DriverStation.Alliance.Blue ? Constants.VisionConstants.kBlueScoreCols : 
                                                  Constants.VisionConstants.kRedScoreCols;

    Pose2d minCol = null;
    double minDist = Double.MAX_VALUE;

    for(int i : scoreCols.keySet()) {
      Pose2d pose = scoreCols.get(i);

      double dy = Math.abs(robotPose.getY() - pose.getY());

      if(dy < minDist) {
        minDist = dy;
        minCol = pose;
      }
    }
    return minCol;
  }

   /**
   * 
   * @param relLoc the target (from camera)
   * @return distance from target
   */
  public static double distFromTag(Transform3d relLoc) {
    return Math.sqrt(relLoc.getX() * relLoc.getX() + 
        relLoc.getY() * relLoc.getY() + 
        relLoc.getZ() * relLoc.getZ());
  }

  /**
   * Log stuff
   */
  public void log() {
    SmartDashboard.putNumber("Robo X", roboPose.getX());
    SmartDashboard.putNumber("Robo Y", roboPose.getY());
  }
}