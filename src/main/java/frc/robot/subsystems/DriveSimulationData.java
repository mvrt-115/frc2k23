// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. 
 * This class maintains simulation and field data as well as heading angle
 * 
*/
public class DriveSimulationData {
    // private AnalogGyro m_gyro = new AnalogGyro(2);
    // private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private SwerveDriveOdometry m_odometry;
    private Field2d m_field2d;
    private double headingAngle;
    public static double currentTime;
    public static double prevTime;

    /**
     * Create a new SimulationData container
     * @param odometry simulation odometry
     * @param field2d
     */
    public DriveSimulationData(SwerveDriveOdometry odometry, Field2d field2d) {
        this.m_odometry = odometry;
        this.m_field2d = field2d;
        headingAngle = 0;
        currentTime = Timer.getFPGATimestamp();
        prevTime = currentTime;
    }

    /**
     * update the simulation odometry and heading with swerve module states and estimated angle (radians)
     * @param moduleStates
     * @param pW position radians
     */
    public void update(SwerveModulePosition[] modulePositions, double pW) {
        headingAngle = pW;
        headingAngle = Math.IEEEremainder(headingAngle, 2*Math.PI);
        SmartDashboard.putNumber("Heading Sim Angle", Math.toDegrees(headingAngle));
        m_odometry.update(new Rotation2d(headingAngle), modulePositions);
        // m_field2d.setRobotPose(m_odometry.getPoseMeters());
        m_field2d.setRobotPose(
            m_odometry.getPoseMeters().getX(), 
            m_odometry.getPoseMeters().getY(),
            new Rotation2d(headingAngle));
    }

    /**
     * pw is position omega 
     * @param dt
     * @param angularVelocity
     * @param modulePositions
     */
    public void quadrature(double angularVelocity, SwerveModulePosition[] modulePositions) {
        double dt = Timer.getFPGATimestamp() - currentTime;
        prevTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("dt", dt);
        double pw = 0;
        if (Math.abs(angularVelocity) > 0.3) {
            pw = angularVelocity * dt + headingAngle;
        }
        update(modulePositions, pw);
    }


    /**
     * Get the simulation heading
     * @return heading in radians
     */
    public double getHeading() {
        return headingAngle;
    }

    /**
     * Set the heading angle
     * @param radians
     */
    public void setHeading(double radians) {
        headingAngle = radians;
    }
}