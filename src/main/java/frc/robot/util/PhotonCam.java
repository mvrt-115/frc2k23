// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.photonvision.PhotonCamera;
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

  public PhotonCam() {
    photonCamera = new PhotonCamera(Constants.VisionConstants.kCameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = photonCamera.getLatestResult();

    for (PhotonTrackedTarget i : result.getTargets()) { // Assume one target for now

      Pose3d roboLocation = ComputerVisionUtil.objectToRobotPose(
          Constants.VisionConstants.aprilTags.get(i.getFiducialId()), i.getAlternateCameraToTarget(),
          new Transform3d());
      //  _____
      // |_____|
      // Origin at bottom left corner of rectangle facing towards the right with CCW
      // being positive
      SmartDashboard.putNumber("x:", roboLocation.getX());
      SmartDashboard.putNumber("y:", roboLocation.getY());
      SmartDashboard.putNumber("z:", roboLocation.getZ());
    }
  }
}
