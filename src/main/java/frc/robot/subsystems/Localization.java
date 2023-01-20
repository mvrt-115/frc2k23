// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// licene deez nuts

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    System.out.println("local periodic");
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

          roboPose = ComputerVisionUtil.objectToRobotPose(tag, relLoc, new Transform3d()).toPose2d();
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

  //  private Pose2d weightTargets(PhotonTrackedTarget ...targets) {
  //     double[] weights = new double[targets.length];

    

  //     return null;
  //  }

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
  public double distFromTag(Transform3d relLoc) {
    return Math.sqrt(relLoc.getX() * relLoc.getX() + 
        relLoc.getY() * relLoc.getY() + 
        relLoc.getZ() * relLoc.getZ());
  }

  /**
   * Log stuff
   */
  public void log() {
    //Log to SmartDashboard
    SmartDashboard.putNumber("Robo X", roboPose.getX());
    SmartDashboard.putNumber("Robo Y", roboPose.getY());

    //Log to AdvantageKit
    Logger.getInstance().recordOutput("Robo X", roboPose.getX());
    Logger.getInstance().recordOutput("Robo Y", roboPose.getY());
  }
}