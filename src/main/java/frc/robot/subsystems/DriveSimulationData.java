// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SimWaitCommand;
import frc.robot.utils.JoystickIO;

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

    private Logger logger;
    private JoystickIO simJoystick = new JoystickIO(2);
    private Pose3d groundIntakePose = new Pose3d(new Translation3d(0.43999999999999995, -0.04, 0.2999999999999998), new Rotation3d(0.0, -Math.PI/2.0, 0.0));
    private Pose3d stage2Pose = new Pose3d(new Translation3d(-0.1538169326817099,0.3400000000000002,0.4434143993148704), new Rotation3d(new Quaternion(-0.5,-0.5,0.5,0.5).toRotationVector())); // new Rotation3d(0.0, Math.PI, Math.PI/2.0)
    // private Pose3d stage3Pose = new Pose3d(new Translation3d(-0.024532990951728118, -0.25000000000000017, 0.35656511234797406), new Rotation3d(new Quaternion(-0.05594691700874833,0.7048900215474868,0.7048900215474866,-0.05594691700874836).toRotationVector()));
    private Pose3d stage3Pose = new Pose3d(new Translation3d(-0.15453299095172823,0.25000000000000017,0.45656511234797414), new Rotation3d(new Quaternion(0.5000000000000003,0.49999999999999994,-0.49999999999999983,-0.5000000000000001)));
    private Pose3d carriagePose = new Pose3d(new Translation3d(-0.3900000000000001,-0.7400000000000005,1.290000000000001), new Rotation3d(new Quaternion(0.5000000000000001,0.5,-0.4999999999999999,-0.5)));
    // private Pose3d offsetPose = new Pose3d(new Translation3d(1, 0, 1), new Rotation3d());
    private Translation3d offsetPose = new Translation3d(0.3, -0.04, 0.2);
    
    private double elev_x = 0;
    private double elev_z = 0;
    private double ground_rot = 0;
    private double elev_angle = Math.toRadians(39.8);
    private static double extention_ratio = 0;
    private static double ground_rot_ratio = -1.0;
    private static double curr_extention_ratio = 0;
    private static double curr_ground_rot_ratio = 0;
    private double kp_elev_sim = 0.1;
    private double kp_ground_sim = 0.1;
    
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
        logger = Logger.getInstance();

        // used when finding zero positions of components
        // simJoystick.button(1).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(0.01, 0, 0), new Rotation3d()))));
        // simJoystick.button(3).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(-0.01, 0, 0), new Rotation3d()))));
        // simJoystick.button(2).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(0, 0.01, 0), new Rotation3d()))));
        // simJoystick.button(4).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(0, -0.01, 0), new Rotation3d()))));
        // simJoystick.button(5).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(0, 0, 0.01), new Rotation3d()))));
        // simJoystick.button(6).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(0, 0, -0.01), new Rotation3d()))));
        // simJoystick.button(7).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(Math.PI/10, 0.0, 0.0)))));
        // simJoystick.button(8).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(-Math.PI/10, 0.0, 0.0)))));
        // simJoystick.button(9).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, Math.PI/10, 0.0)))));
        // simJoystick.button(10).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, -Math.PI/10, 0.0)))));
        // simJoystick.button(11).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI/10)))));
        // simJoystick.button(12).onTrue(new InstantCommand(() -> stage3Pose = stage3Pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, -Math.PI/10)))));

        // simJoystick.button(1).onTrue(new InstantCommand(() -> offsetPose = offsetPose.plus(new Translation3d(0.01, 0, 0))));
        // simJoystick.button(3).onTrue(new InstantCommand(() -> offsetPose = offsetPose.plus(new Translation3d(-0.01, 0, 0))));
        // simJoystick.button(2).onTrue(new InstantCommand(() -> offsetPose = offsetPose.plus(new Translation3d(0, 0, 0.01))));
        // simJoystick.button(4).onTrue(new InstantCommand(() -> offsetPose = offsetPose.plus(new Translation3d(0, 0, -0.01))));

        simJoystick.button(1).onTrue(new InstantCommand(() -> setElevator(1)));
        simJoystick.button(3).onTrue(new InstantCommand(() -> setElevator(0.5)));
        simJoystick.button(4).onTrue(new InstantCommand(() -> setElevator(0.5)));
        simJoystick.button(2).onTrue(new InstantCommand(() -> setElevator(0)));

        simJoystick.button(6).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setElevator(0.4)),
                new SimWaitCommand(0.5),
                new InstantCommand(() -> setGroundRot(1)),
                new SimWaitCommand(0.5),
                new InstantCommand(() -> setElevator(0))
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> setElevator(0.4)),
                new SimWaitCommand(0.5),
                new InstantCommand(() -> setGroundRot(0)),
                new SimWaitCommand(0.5),
                new InstantCommand(() -> setElevator(0))
            )
        );
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
            m_odometry.getPoseMeters().getRotation());
        
        // elev_x = Math.max(0, simJoystick.getX()) * 0.55 * Math.cos(elev_angle);
        // elev_z = Math.max(0, simJoystick.getX()) * 0.55 * Math.sin(elev_angle);
        // ground_rot = 0.83 * Math.PI * (Math.min(simJoystick.getRawAxis(1), 0));

        curr_extention_ratio += (extention_ratio - curr_extention_ratio) * kp_elev_sim;
        curr_ground_rot_ratio += (ground_rot_ratio - curr_ground_rot_ratio) * kp_ground_sim;

        elev_x = Math.max(0, curr_extention_ratio) * 0.55 * Math.cos(elev_angle);
        elev_z = Math.max(0, curr_extention_ratio) * 0.55 * Math.sin(elev_angle);
        ground_rot = 0.83 * Math.PI * (Math.min(curr_ground_rot_ratio, 0));

        Rotation3d R = new Rotation3d(0.0, ground_rot, 0.0);
        Pose3d newGroundPose = rotateAroundPoint(groundIntakePose, R, offsetPose);

        // T(x)=R(x−v)+v=Rx+(v−Rv)
        // Transform3d rT = new Transform3d(new Translation3d(), R);
        // Pose3d newGroundPose = groundIntakePose.transformBy(rT).plus(offsetPose.minus(offsetPose.transformBy(rT)));
        // Pose3d newGroundPose = groundIntakePose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, ground_rot, 0.0)));


        logger.recordOutput("GroundIntakeComponent", newGroundPose); // .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, ground_rot, 0.0)))
        logger.recordOutput("ground axis", groundIntakePose);
        logger.recordOutput("Stage2Component", stage2Pose.transformBy(new Transform3d(new Translation3d(0.0, elev_z, -elev_x), new Rotation3d())));
        logger.recordOutput("Stage3Component", stage3Pose.transformBy(new Transform3d(new Translation3d(0.0, 2*elev_z, -2*elev_x), new Rotation3d())));
        logger.recordOutput("IntakeComponent", carriagePose.transformBy(new Transform3d(new Translation3d(0.0, 3*elev_z, -3*elev_x), new Rotation3d())));
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
        if (Math.abs(angularVelocity) > 0.0005) {
            pw = angularVelocity * dt + headingAngle;
        }
        else {
            pw = headingAngle;
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

    /**
     * reset odometry
     * @param rotation2d
     * @param modulePositions
     * @param pose
     */
    public void resetOdometry(Rotation2d rotation2d, SwerveModulePosition[] modulePositions, Pose2d pose) {
        m_odometry.resetPosition(rotation2d, modulePositions, pose);
        // headingAngle = pose.getRotation().getRadians();
    }

    /**
     * Rotate a point around a specified point in space by a rotation3d
     * @param p original point
     * @param R specified rotation
     * @param o point around which to rotate
     * @return transformed point
     */
    public static Pose3d rotateAroundPoint(Pose3d p, Rotation3d R, Translation3d o) {
        return new Pose3d(p.getTranslation().minus(o).rotateBy(R).plus(o), p.getRotation().rotateBy(R));
    }

    /**
     * Set the elevator pose (scaled as a fraction of max extention 0 to 1)
     * @param frac
     */
    public static void setElevator(double frac) {
        extention_ratio = frac;
    }

    /**
     * Set the groundintake pose (scaled as a fraction of max rotation 0 to 1)
     * @param frac
     */
    public static void setGroundRot(double frac) {
        ground_rot_ratio = -1 + frac;
    }
}