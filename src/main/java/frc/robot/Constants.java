// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Elevator.ElevatorState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static class DataLogging {
                public static final Mode currMode = RobotBase.isSimulation() ? Mode.SIM : Mode.REAL;

                public static enum Mode {
                        REAL, REPLAY, SIM
                }

        }

        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
        }

        public static class JoystickControls {
                public static final boolean xBoxControl = false;
                public static final boolean invertJoystickX = true;
                public static final boolean invertJoystickY = true;
                public static final boolean invertJoystickW = true;

                public static final double kPJoystick = 0.001;
                public static final double kIJoystick = 0.0;
                public static final double kDJoystick = 0.0;
                public static final double kFJoystick = 0.0;

        }

        public static class SwerveDrivetrain {
                // Physical Constants
                public static final double chassisWidth = Units.inchesToMeters(28);
                public static final double chassisLength = Units.inchesToMeters(26);// swap for comp

                // Important locations for swerve
                // consider swapping corners
                public static final Translation2d m_standardCenterLocation = new Translation2d(0, 0);
                public static final Translation2d m_frontLeftLocation = new Translation2d(chassisLength / 2.0,
                                chassisWidth / 2.0);
                public static final Translation2d m_frontRightLocation = new Translation2d(chassisLength / 2.0,
                                -chassisWidth / 2.0);
                public static final Translation2d m_backLeftLocation = new Translation2d(-chassisLength / 2.0,
                                chassisWidth / 2.0);
                public static final Translation2d m_backRightLocation = new Translation2d(-chassisLength / 2.0,
                                -chassisWidth / 2.0);
                public static final Translation2d[] rotatePoints = {
                                m_standardCenterLocation,
                                m_frontLeftLocation,
                                m_frontRightLocation,
                                m_backLeftLocation,
                                m_backRightLocation
                };

                // Leveling PID
                public static final double levelkP = 0.025;
                public static final double levelkI = 0;
                public static final double levelkD = 0;
                public static final double rotatekP = 0;
                public static final double rotatekI = 0;
                public static final double rotatekD = 0;
                public static final double angleTolerance = 1.5;
                public static final double speedTolerance = 0.5;
                public static final double levelVelocityMPS = 0.75;
                public static final double yawAlligned = 0; // TBD
                public static final double pitchAlligned = 0; // TBD
                public static final double rollAlligned = 0; // TBD

                public static final double maxAngularSpeed = 0;

                // Motor ID
                public static final int m_frontRightDriveID = 1;
                public static final int m_frontLeftDriveID = 3;
                public static final int m_backLeftDriveID = 5;
                public static final int m_backRightDriveID = 7;

                public static final int m_frontRightTurnID = 2;
                public static final int m_frontLeftTurnID = 4;
                public static final int m_backLeftTurnID = 6;
                public static final int m_backRightTurnID = 8;

                // Abs Encoder ID
                public static final int m_frontRightEncoderID = 9;
                public static final int m_frontLeftEncoderID = 10;
                public static final int m_backLeftEncoderID = 11;
                public static final int m_backRightEncoderID = 12;

                // Comp Bot Encoder Offsets
                public static final boolean isCompBot = false;

                public static final double m_frontLeftEncoderOffset_Comp = Units.degreesToRadians(25.31);// +
                                                                                                         // Math.PI/2.0;
                public static final double m_frontRightEncoderOffset_Comp = Units.degreesToRadians(317.02);// +
                                                                                                           // Math.PI/2.0;
                public static final double m_backLeftEncoderOffset_Comp = Units.degreesToRadians(37.00);// +
                                                                                                        // Math.PI/2.0;
                public static final double m_backRightEncoderOffset_Comp = Units.degreesToRadians(126.38);// +
                                                                                                          // Math.PI/2.0;

                // Practice Bot Encoder Offsets
                public static final double m_frontLeftEncoderOffset_P = Units.degreesToRadians(33.62);// + Math.PI/2.0;
                public static final double m_frontRightEncoderOffset_P = Units.degreesToRadians(182.180);// +
                                                                                                         // Math.PI/2.0;
                public static final double m_backLeftEncoderOffset_P = Units.degreesToRadians(341.54);// + Math.PI/2.0;
                public static final double m_backRightEncoderOffset_P = Units.degreesToRadians(146.86);// + Math.PI/2.0;

                // Abs Encoder Offsets
                public static final double m_frontLeftEncoderOffset = isCompBot ? m_frontLeftEncoderOffset_Comp
                                : m_frontLeftEncoderOffset_P;// + Math.PI/2.0;
                public static final double m_frontRightEncoderOffset = isCompBot ? m_frontRightEncoderOffset_Comp
                                : m_frontRightEncoderOffset_P;// + Math.PI/2.0;
                public static final double m_backLeftEncoderOffset = isCompBot ? m_backLeftEncoderOffset_Comp
                                : m_backLeftEncoderOffset_P;// + Math.PI/2.0;
                public static final double m_backRightEncoderOffset = isCompBot ? m_backRightEncoderOffset_Comp
                                : m_backRightEncoderOffset_P;// + Math.PI/2.0;

                // constants for joystick drive
                public static final double kSensitivity = 0.90;
                public static final double kWheelDeadband = 0.2;
                public static final double kThrottleDeadband = 0.2;
                public static final double kWheelGain = 0.05;
                public static final double kWheelNonlinearity = 0.05;
                public static final double kMaxSpeedMPS = 10; // optimize max speed to prioritize translation
                public static final double kDriveMaxAcceleration = 3;
                public static final double kTurnMaxAcceleration = 1 * Math.PI;
                public static final double kDriveMaxSpeedMPS = 7;
                public static final double kTurnMaxSpeedRPS = 4 * Math.PI;
                public static final int kDriveJoystickPort = 0;
                public static final int kDriveXAxis = 0;
                public static final int kDriveYAxis = 1;
                public static final int kDriveWAxis = 4;
                public static final int kDriveFieldOrientButtonIdx = 1;

                // values to be determined after the robot is characterized
                public static final double kS = 0; // 0.69382 //units: Volts
                public static final double kV = 0; // 1.30485 //2.6097 //units: Volts * Seconds / Meters
                public static final double kA = 0; // 0.35228 //units: Volts * Seconds^2 / Meters

                // Position PID
                public static final double m_x_control_P = 2.0;
                public static final double m_x_control_I = 0.5;
                public static final double m_x_control_D = 0.0;
                public static final double m_y_control_P = 2.0;
                public static final double m_y_control_I = 0.5;
                public static final double m_y_control_D = 0.0;
                public static final double m_r_control_P = 2.0;
                public static final double m_r_control_I = 0.0;
                public static final double m_r_control_D = 0;

                // Auton Constants
                public static final double kMaxAutonDriveSpeed = 4; // mps
                public static final double kMaxAutonDriveAcceleration = 3; // mps2
                public static final double kMaxAutonThetaVelocity = kMaxAutonDriveSpeed
                                / Math.hypot(chassisWidth / 2.0, chassisLength / 2.0); // rad ps
                public static final double kMaxAutonThetaAcceleration = kMaxAutonDriveAcceleration
                                / Math.hypot(chassisWidth / 2.0, chassisLength / 2.0); // rad ps^2

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAutonThetaVelocity, kMaxAutonThetaAcceleration);

                public static final double kTeleopHeadingCorrectionScale = 0;
        }

        public static class Talon {
                public static final int talonFXTicks = 2048;
                public static final int talonSRXTicks = 4096;

                public static final double MAX_VOLTAGE = 10.0;

                public static final int kPIDIdx = 0;
                public static final int kTimeoutMs = 10;
                public static final boolean kIsPracticeBot = false;
                public static final double kVoltageComp = 10.0;
                public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(
                                true,
                                40,
                                50, 3.8);

        }

        public static class SwerveModule {
                public static final double gear_ratio_turn = 150.0 / 7.0; // number of rotations of talon for one turn
                                                                          // of wheel
                public static final double gear_ratio_drive = 6.75 / 1.0; // number of rotations of talon for one
                                                                          // rotation of
                                                                          // wheel
                public static final double radius = 0.05; // meters
                public static final double kwheelCircumference = 2 * Math.PI * radius; // meters

                // PID Constants
                public static final double kP = 0.1;// 0.3; // 3.2364;
                public static final double kI = 0;// 0.001;
                public static final double kD = 0;
                public static final double kFF = 0;

                public static final double kPTurn = 0.3;
                public static final double kITurn = 0.0;
                public static final double kDTurn = 0.0;
                public static final double kFTurn = 0.0;
        }

        public static class Elevator {
                public static final int MOTOR_ID = 13;
                public static final int MOTOR_ID2 = 14;

                public static final int kPIDIdx = 0;
                public static final double P = 0.025;// .1;//.01;
                public static final double I = 0;// .000006;
                public static final double D = 0.007;
                public static final double F = 0;

                // Wtvr it is
                public static final double METERS_PER_TICK = .500;
                public static final double INCHES_PER_TICK = 10.6224;

                // Min/Max heights for the elevator (in inches)
                public static final double MAX_HEIGHT = 55;
                public static final double MIN_HEIGHT = 0;

                public static final double ZERO_HEIGHT = 0;
                public static final double SHELF_HEIGHT = 0;

                // MID, HIGH heights parwa cone (in ticks)
                public static final double CONE_MID_HEIGHT = 17000;
                public static final double CONE_HIGH_HEIGHT = 22600;
                public static final double INTAKE_HEIGHT = 15700;

                // MID, HIGH heights para cube (in inches)
                public static final double CUBE_MID_HEIGHT = 15000;
                public static final double CUBE_HIGH_HEIGHT = 21000;

                // feed forward constants
                public static final double kS = -0.086653;// -0.55996;//-0.086653;//-0.55996;
                // public static final double kG = 1;
                public static final double kG = 0.67635; // 0.79635: claw intake; //1.2265;
                public static final double kV = 0.016763; // 0.035258;
                public static final double kA = 0.0031226; // 0.0053228;
                // public static final double kA = 0;
                // Game Object Heights
                public static final double CONE_HEIGHT = 6;
                public static final double CUBE_HIEGHT = 8;

                public static final int SENSOR_PORT = 0;
                public static final double KDt = 0.01;

                // constraints
                public static final double MAX_VELOCITY = 14; //50;
                public static final double MAX_ACCELERATION = 4; //25;

                // initial elevator stages
                public static final ElevatorState TELEOP_INIT_STATE = ElevatorState.ZEROED;

                public static final double ERROR = 50;

                public static final double GEAR_RATIO = 3;

                public static final double MASS = 8;
                public static final double BOTTOM = 0;
                public static final double MIDDLE = 10;
                public static final double TOP = 15;

                public static final double PULLEY_RADIUS = 2;

        }

        public static class VisionConstants {
                public static final String kCamera1Name = "bettygotmoney" + "cam"; // neg offset
                public static final String kCamera2Name = "bohm" + "cam"; // pos offset

                public static final double minDistFromTag = 0.3; // Min dist necessary from tag to automate (0.3 meter
                                                                 // aprox)
                public static final double xyTolerance = 0.05;
                public static final double thetaTolerance = 0.05;

                // Camera position on robot
                public static final Transform3d cam1ToRobot = new Transform3d(
                                new Translation3d(0, -(4.5 * 2.54) / 100.0 - 0.05, 0), new Rotation3d());
                public static final Transform3d cam2ToRobot = new Transform3d(
                                new Translation3d(0, (4.5 * 2.54) / 100.0 + 0.05, 0), new Rotation3d());
                /**
                 * Key:
                 * Orientation: facing red community from blue community
                 * https://cdn.discordapp.com/attachments/453058111893405727/1062210473900126238/Screen_Shot_2023-01-09_at_7.25.00_PM.png
                 * Red Alliance Scoring Locations (right to left) – IDs 1, 2, ...9
                 * Blue Alliance Scoring Locations (left to right) – IDs 10, 11, ...18
                 */
                public static final Map<Integer, Pose2d> kRedScoreCols = Map.of(
                                1,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(20.6),
                                                new Rotation2d(Math.PI)),
                                2,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(41.9),
                                                new Rotation2d(Math.PI)),
                                3,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(64.6),
                                                new Rotation2d(Math.PI)),
                                4,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(86.5),
                                                new Rotation2d(Math.PI)),
                                5,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(108.9),
                                                new Rotation2d(Math.PI)),
                                6,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(130.6),
                                                new Rotation2d(Math.PI)),
                                7,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(157.6),
                                                new Rotation2d(Math.PI)),
                                8,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(174.9),
                                                new Rotation2d(Math.PI)),
                                9,
                                new Pose2d(
                                                Units.inchesToMeters(597.1),
                                                Units.inchesToMeters(196.6),
                                                new Rotation2d(Math.PI)));

                public static final Map<Integer, Pose2d> kBlueScoreCols = Map.of(
                                9,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(196.6),
                                                new Rotation2d(0)),
                                8,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(174.9),
                                                new Rotation2d(0)),
                                7,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(157.6),
                                                new Rotation2d(0)),
                                6,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(130.6),
                                                new Rotation2d(0)),
                                5,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(108.9),
                                                new Rotation2d(0)),
                                4,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(86.5),
                                                new Rotation2d(0)),
                                3,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(64.6),
                                                new Rotation2d(0)),
                                2,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(41.9),
                                                new Rotation2d(0)),
                                1,
                                new Pose2d(
                                                Units.inchesToMeters(56.4),
                                                Units.inchesToMeters(20.6),
                                                new Rotation2d(0)));

                /**
                 * Key:
                 * Orientation: Facing red community from blue community
                 * https://cdn.discordapp.com/attachments/453058111893405727/1062210473900126238/Screen_Shot_2023-01-09_at_7.25.00_PM.png
                 * Red Alliance Community (right to left) – IDs 1, 2, 3
                 * Blue Alliance Double Substation – ID 4
                 * Red Alliance Double Substation – ID 5
                 * Blue Alliance Community (left to right) – IDs 6, 7, 8
                 */
                public static final Map<Integer, Pose3d> aprilTags = // relative to corner closest to tag 8
                                Map.of(
                                                1,
                                                new Pose3d(
                                                                Units.inchesToMeters(610.77),
                                                                Units.inchesToMeters(42.19),
                                                                Units.inchesToMeters(18.22),
                                                                new Rotation3d(0.0, 0.0, Math.PI)),
                                                2,
                                                new Pose3d(
                                                                Units.inchesToMeters(610.77),
                                                                Units.inchesToMeters(108.19),
                                                                Units.inchesToMeters(18.22),
                                                                new Rotation3d(0.0, 0.0, Math.PI)),
                                                3,
                                                new Pose3d(
                                                                Units.inchesToMeters(610.77),
                                                                Units.inchesToMeters(174.19), // FIRST's diagram has a
                                                                                              // typo (it says 147.19)
                                                                Units.inchesToMeters(18.22),
                                                                new Rotation3d(0.0, 0.0, Math.PI)),
                                                4,
                                                new Pose3d(
                                                                Units.inchesToMeters(636.96),
                                                                Units.inchesToMeters(265.74),
                                                                Units.inchesToMeters(27.38),
                                                                new Rotation3d(0.0, 0.0, Math.PI)),
                                                5,
                                                new Pose3d(
                                                                Units.inchesToMeters(14.25),
                                                                Units.inchesToMeters(265.74),
                                                                Units.inchesToMeters(27.38),
                                                                new Rotation3d()),
                                                6,
                                                new Pose3d(
                                                                Units.inchesToMeters(40.45),
                                                                Units.inchesToMeters(174.19), // FIRST's diagram has a
                                                                                              // typo (it says 147.19)
                                                                Units.inchesToMeters(18.22),
                                                                new Rotation3d()),
                                                7,
                                                new Pose3d(
                                                                Units.inchesToMeters(40.45),
                                                                Units.inchesToMeters(108.19),
                                                                Units.inchesToMeters(18.22),
                                                                new Rotation3d()),
                                                8,
                                                new Pose3d(
                                                                Units.inchesToMeters(40.45),
                                                                Units.inchesToMeters(42.19),
                                                                Units.inchesToMeters(18.22),
                                                                new Rotation3d()));
        }

        public static class Intake {
                public static final double kMarginOfError = 0.03;

                public static final int kProximityPort = 6; // port number for element proximity sensor
                public static final int kMotorPort = 5;

                public static final double kP = 0, kI = 0, kD = 0;

                public static final double kCompressedSpeed = 0.15;

                public static final double kGoalRPM = -0.4;
                public static final double kOuttakeRPM = 0.05;// 0.08;
        }
}
