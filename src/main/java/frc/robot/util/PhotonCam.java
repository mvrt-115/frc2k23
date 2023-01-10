package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PhotonCam {
 
    private PhotonCamera photonCamera;
    private Pose3d tagLocation = new Pose3d(10, 5, 0, new Rotation3d());

    public PhotonCam(){

    }

    public void logXYZ(){
        CommandScheduler.getInstance().run();

        // This method will be called once per scheduler run
        var result = photonCamera.getLatestResult();

        for(PhotonTrackedTarget i : result.getTargets()){ //Assume one target for now
        Pose3d roboLocation = ComputerVisionUtil.objectToRobotPose(tagLocation, i.getAlternateCameraToTarget(), new Transform3d());

        // _____ 
        //|_____|
        //Origin at bottom left corner of rectangle facing towards the right with CCW being positive
        SmartDashboard.putNumber("x:", roboLocation.getX());
        SmartDashboard.putNumber("y:", roboLocation.getY());
        SmartDashboard.putNumber("z:", roboLocation.getZ());
    }
    }
    
}