// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.PhotonPoseEstimator;
import frc.robot.utils.PhotonPoseEstimator.PoseStrategy;

public class LocalizationTest extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPoseEstimator cameraEstimator;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private SwerveDrivetrain swerveDrivetrain;
  private AprilTagFieldLayout fieldLayout;
  public static final Transform3d cam2ToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(14), Units.inchesToMeters(-1), 0), new Rotation3d(Units.degreesToRadians(90), Units.degreesToRadians(0), Units.degreesToRadians(15)));
  private Pose2d curLoc;
  private Logger logger = Logger.getInstance();
  /** Creates a new LocalizationTest. */
  public LocalizationTest(SwerveDrivetrain sd) {
    swerveDrivetrain = sd;
    camera = new PhotonCamera("beholder");
    try {
      this.fieldLayout = AprilTagFieldLayout.loadFromResource("");
    } catch (Exception e) {
      //
    }
    cameraEstimator = new PhotonPoseEstimator(this.fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, cam2ToRobot);
    cameraEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDrivetrain.getKinematics(), swerveDrivetrain.getPose().getRotation(), swerveDrivetrain.getModulePositions(), new Pose2d());
  }

  public Pose2d getCurrentPosition() {
    // Optional<EstimatedRobotPose> result = cameraEstimator.update();
    // Pose3d pose = result.get().estimatedPose;
    // return new Pose2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d(pose.getRotation().getAngle()));
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }
  public Pose2d getTargetPosition() {
    Map<Integer, Pose2d> scoringLocations;
    if (DriverStation.getAlliance() == Alliance.Red) {
      scoringLocations = Constants.VisionConstants.kRedScoreCols;
    } else {
      scoringLocations = Constants.VisionConstants.kBlueScoreCols;
    }
    int closestCol = -1;
    Pose2d curLoc = getCurrentPosition();
    double closestDist = 1000000;
    for (int col = 1; col <= 9; col++) {
      // get distance from curLoc
      if (scoringLocations.get(col).getY() - curLoc.getY() < closestDist) {
        closestDist = Math.abs(scoringLocations.get(col).getY() - curLoc.getY());
        closestCol = col;
      }
    }
    return scoringLocations.get(closestCol);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = getCurrentPosition();
    Pose2d poseToGoTo = getTargetPosition();
    swerveDrivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerveDrivetrain.getPose().getRotation(), swerveDrivetrain.getModulePositions());
    logger.recordOutput("Vision/error_x", poseToGoTo.getX() - robotPose.getX());
    logger.recordOutput("Vision/error_y", poseToGoTo.getY() - robotPose.getY());
    //logger.recordOutput("Vision/error_theta", poseToGoTo.getRotation().getDegrees() - computeThetaError(robotPose.getRotation().getDegrees(), false));
    logger.recordOutput("Vision/goalPose", poseToGoTo);
    // This method will be called once per scheduler run
    // Optional<EstimatedRobotPose> result = cameraEstimator.update();
    // SmartDashboard.putNumber("distanceX", result.get().estimatedPose.getX());
    // SmartDashboard.putNumber("distanceY", result.get().estimatedPose.getY());
    // SmartDashboard.putNumber("theta", result.get().estimatedPose.getRotation().getAngle());
  }
}
