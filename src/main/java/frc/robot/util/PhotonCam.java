// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
      SmartDashboard.putBoolean("connected", true);

    } else {
      SmartDashboard.putBoolean("connected", false);
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
   * @param relLoc the target
   * @return distance from target
   */
  public double distFromTag(Transform3d relLoc) {
    return Math.sqrt(relLoc.getX() * relLoc.getX() + 
        relLoc.getY() * relLoc.getY() + 
        relLoc.getZ() * relLoc.getZ());
  }

  /**
   * Log stuff
   * @param pose the pose of target
   */
  public void log(){
    SmartDashboard.putNumber("Tag Robo X", roboPose.getX());
    SmartDashboard.putNumber("Tag Robo Y", roboPose.getY());
    SmartDashboard.putNumber("Tag Robo Z", roboPose.getZ());
  }
}