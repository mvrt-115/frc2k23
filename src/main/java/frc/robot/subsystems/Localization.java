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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

  public Localization(SwerveDrivetrain swerveDrivetrain) {
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

    //Tags exist
    if (result.hasTargets()){
      roboPose = getCurrentPose();
      log();
      SmartDashboard.putBoolean("Found Tag(s)", true);

    } else {
      SmartDashboard.putBoolean("Found Tag(s)", false);
    }

    var res = camera.getLatestResult();

    Map<Integer, Pose3d> targetPoses = Constants.VisionConstants.aprilTags;

    if (res.hasTargets()) {
      double imageCaptureTime = Timer.getFPGATimestamp() - (res.getLatencyMillis() / 1000d);

      //Loop through received objects
      for (PhotonTrackedTarget target : res.getTargets()) {
        int fiducialId = target.getFiducialId();

        if (fiducialId >= 0 && fiducialId < targetPoses.size()) {
          Pose3d targetPose = targetPoses.get(fiducialId);

          Transform3d relLoc = target.getBestCameraToTarget();
          Pose3d tag = Constants.VisionConstants.aprilTags.get(target.getFiducialId());

          roboPose = ComputerVisionUtil.objectToRobotPose(tag, relLoc, new Transform3d());

          Pose2d roboOnField = new Pose2d(roboPose.getX(), roboPose.getY(), swerveDrivetrain.getRotation2d());
          //This may be sketchy

          field.getObject("MyRobot" + fiducialId).setPose(roboOnField);
          poseEstimator.addVisionMeasurement(roboOnField, imageCaptureTime);
        }
      }
    }
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerveDrivetrain.getRotation2d(), swerveDrivetrain.getModulePositions());

    field.setRobotPose(getCurrentPose());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** _____
   * |_____|
   * Origin at bottom left corner of rectangle facing towards the right with CCW
   * being positive
   *
   * @return Estimated pose of robot based on closest detected AprilTag
   */
  public Pose3d getEstimatedPose() { // TODO: change to Pose2d
    PhotonPipelineResult result = camera.getLatestResult();
   
    //Best target
    PhotonTrackedTarget target = result.getBestTarget();
   
    if (target != null){
      Transform3d relLoc = target.getBestCameraToTarget();
      Pose3d tag = Constants.VisionConstants.aprilTags.get(target.getFiducialId());

      SmartDashboard.putNumber("Tag Distance", distFromTag(relLoc));
      
      return ComputerVisionUtil.objectToRobotPose(tag, relLoc, new Transform3d());
    }

    return null;
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
   * Gets the closest scoring col from the robot's pose (centered at the camera)
   * @param robotPose The robot pose
   * @return Returns the Pose3d of the scoring col
   */
  public Pose2d getClosestScoringLoc(Pose2d robotPose) {
    Map<Integer, Pose2d> scoreCols = new HashMap<Integer, Pose2d>();

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
   * Log stuff
   */
  public void log() {
    SmartDashboard.putNumber("Robo X", roboPose.getX());
    SmartDashboard.putNumber("Robo Y", roboPose.getY());
  }
}