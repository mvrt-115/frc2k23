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
  /** Creates a new PhotonCam. */
  private PhotonCamera photonCamera;

  public PhotonCam(PhotonCamera photonCamera) {
    // photonCamera = new PhotonCamera(Constants.VisionConstants.kCameraName);
    this.photonCamera = photonCamera;
  }

  @Override
  public void periodic() {
    Pose3d pose = getEstimatedPose();

    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Z", pose.getZ());
  }

  /**
   *  _____
   * |_____|
   * Origin at bottom left corner of rectangle facing towards the right with CCW
   * being positive
   *
   * @return Estimated pose of robot based on closest detected AprilTag
   */
  public Pose3d getEstimatedPose() {
    PhotonPipelineResult result = photonCamera.getLatestResult();

    Pose3d pose = null;

    double minDist = Double.MAX_VALUE;
    for (PhotonTrackedTarget i : result.getTargets()) { // Assume one target for now
      Transform3d relLoc = i.getAlternateCameraToTarget();
      Pose3d tag = Constants.VisionConstants.aprilTags.get(i.getFiducialId());

      double dist = Math.sqrt((relLoc.getX() - tag.getX()) * (relLoc.getX() - tag.getX()) + 
        (relLoc.getY() - tag.getY()) * (relLoc.getY() - tag.getY()));

      if(dist < minDist) {
        minDist = dist;

        pose = ComputerVisionUtil.objectToRobotPose(tag, relLoc, new Transform3d());
      }
    }

    return pose;
  }
}
