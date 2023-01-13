// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// licene deez nuts

package frc.robot.utils;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// penis
import frc.robot.RobotContainer;

public class PhotonCam extends SubsystemBase {
  private PhotonCamera photonCamera;
  private Pose3d roboPose;

  public PhotonCam() {
    photonCamera = new PhotonCamera(Constants.VisionConstants.kCameraName);
    roboPose = new Pose3d();
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = photonCamera.getLatestResult();

    //Tags exist
    if (result.hasTargets()){
      roboPose = getEstimatedPose();
      log();
      SmartDashboard.putBoolean("Found Tag(s)", true);

    } else {
      SmartDashboard.putBoolean("Found Tag(s)", false);
    }
  }

  /** _____
   * |_____|
   * Origin at bottom left corner of rectangle facing towards the right with CCW
   * being positive
   *
   * @return Estimated pose of robot based on closest detected AprilTag
   */
  public Pose3d getEstimatedPose() {
    PhotonPipelineResult result = photonCamera.getLatestResult();
   
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
  public Pose3d getClosestScoringLoc(Pose3d robotPose) {
    Map<Integer, Pose3d> scoreCols = new HashMap<Integer, Pose3d>();

    Pose3d minCol = null;
    double minDist = Double.MAX_VALUE;

    for(int i : scoreCols.keySet()) {
      Pose3d pose = scoreCols.get(i);

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
    SmartDashboard.putNumber("Robo Z", roboPose.getZ());
  }
}