// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        public static final boolean isCompBot = true;

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

                // //red
                // public static final boolean invertJoystickX = false;
                // public static final boolean invertJoystickY = false;
                // public static final boolean invertJoystickW = true;
                
                public static final double kPJoystick = 0.001;
                public static final double kIJoystick = 0.0;
                public static final double kDJoystick = 0.0;
                public static final double kFJoystick = 0.0;

        }
        public static class Intake {
                public static final double kMarginOfError = 0.03;

                public static final int kProximityPort = 6; // port number for element proximity sensor
                public static final int kMotorPort = 16;

                public static final double kP = 0, kI = 0, kD = 0;

                public static final double kCompressedSpeed = isCompBot? -0.35 : -0.07;

                public static final double kGoalRPM = isCompBot ? -0.4 : 0.3;

                public static final double kOuttakeRPM = isCompBot?  0.0725 :  0.08;

                public static final double outtakeCube = 0.15;
        }

        public static class GroundIntake {
                // public static final double kMarginOfError = 0.03;

                // public static final int kProximityPort = 6; // port number for element proximity sensor
                public static final int kPivotId = 11;//isCompBot? 8 : 5;
                public static final int kRollerId = 15;//isCompBot? 8 : 5;

                public static final double kRollerOutput = 0.5;

                public static final double kP = 0.05;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kG = 0.22808;
                public static final double initialAngle = 30.0;
                public static final double kGearRatio = 30.0;

        }

        public static class Leveling {
                // Leveling PID
                public static final double levelkP = 0.03; // 0.05
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

                // DriveForward
                public static final double driveForwardMPS = 4;
                public static final double driveForwardTime = 1.25;
        }
        public static class SwerveDrivetrain {
                // Physical Constants
                public static final double chassisWidth = Units.inchesToMeters(26);
                public static final double chassisLength = Units.inchesToMeters(28);// swap for comp

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
                public static final boolean isCompBot = true;
                public static final String canivore_name = (isCompBot)? "Drivetrain" : "rio";

                public static final double m_frontLeftEncoderOffset_Comp = Units.degreesToRadians(222.802); // 128.32// + Math.PI/2.0;
                public static final double m_frontRightEncoderOffset_Comp = Units.degreesToRadians(273.602); // 115.04// + Math.PI/2.0;
                public static final double m_backLeftEncoderOffset_Comp = Units.degreesToRadians(212.607); //214.8 // + Math.PI/2.0;
                public static final double m_backRightEncoderOffset_Comp = Units.degreesToRadians(306.56); // 341.81// + Math.PI/2.0;

                // Practice Bot Encoder Offsets
                public static final double m_frontLeftEncoderOffset_P = Units.degreesToRadians(36.21);// + Math.PI/2.0; //33.62
                public static final double m_frontRightEncoderOffset_P = Units.degreesToRadians(87.89);// + Math.PI/2.0; //182.180
                public static final double m_backLeftEncoderOffset_P = Units.degreesToRadians(212.51);// + Math.PI/2.0; //341.54
                public static final double m_backRightEncoderOffset_P = Units.degreesToRadians(314.38);// + Math.PI/2.0; //304.277

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

                public static final double kMaxSpeedMPS = 10; //3; // optimize max speed to prioritize translation
                public static final double kDriveMaxAccelerationNormal = 3; //2;
                public static final double kTurnMaxAccelerationNormal = 0.5 * Math.PI;
                public static final double kDriveMaxSpeedMPSNormal = 4;
                public static final double kTurnMaxSpeedRPSNormal = 1 * Math.PI;
                public static final double kDriveMaxSpeedCap = 10;
                public static double kDriveMaxSpeedMPS = kDriveMaxSpeedMPSNormal;
                public static double kTurnMaxSpeedRPS = kTurnMaxSpeedRPSNormal;
                public static double kDriveMaxAcceleration = kDriveMaxAccelerationNormal;
                public static double kTurnMaxAcceleration = kTurnMaxAccelerationNormal;
                
                public static final int kDriveJoystickPort = 0;
                public static final int kDriveXAxis = 0;
                public static final int kDriveYAxis = 1;
                public static final int kDriveWAxis = 4;
                public static final int kDriveFieldOrientButtonIdx = 8;
                public static final int kDriveLeftTrigger = 2;
                public static final int kDriveRightTrigger = 3;

                // values to be determined after the robot is characterized
                public static final double kS = 0; // 0.69382 //units: Volts
                public static final double kV = 0; // 1.30485 //2.6097 //units: Volts * Seconds / Meters
                public static final double kA = 0; // 0.35228 //units: Volts * Seconds^2 / Meters

                // Position PID
                public static final double m_x_control_P = 1.6;
                public static final double m_x_control_I = 0.5;
                public static final double m_x_control_D = 0.0;
                public static final double m_y_control_P = 1.6;
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
                                35,
                                50, 0.75);

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
                public static final double P = 0.0237; //prac bot: 0.025;// .1;//.01;
                public static final double I = 0;// .000006;
                public static final double D = 0.13; //0.1;
                public static final double F = 0;

                // Wtvr it is
                // public static final double METERS_PER_TICK = .500;
                // public static final double INCHES_PER_TICK = 10.6224;

                // Min/Max heights for the elevator (in inches)
                public static final double MAX_HEIGHT = 57;
                public static final double MIN_HEIGHT = 0;

                public static final double ZERO_HEIGHT = 0;
                public static final double SHELF_HEIGHT = 0;

                // MID, HIGH heights parwa cone (in ticks)
                public static final double CONE_MID_HEIGHT = 38;
                public static final double CONE_HIGH_HEIGHT = 58.6;
                public static final double INTAKE_HEIGHT = 39.6;

                // MID, HIGH heights para cube (in inches)
                public static final double CUBE_MID_HEIGHT = 32;
                public static final double CUBE_HIGH_HEIGHT = 49;

                // feed forward constants
                public static final double kS = isCompBot ? 0.12073 : 0.38161; // oggg 0.15639 //-0.086653;// -0.55996;//-0.086653;//-0.55996;
                // public static final double kG = 1;
                public static final double kG = isCompBot ? 0.53487 : 0.79635; //old good one 0.48098 //0.87635; // 0.79635: claw intake; //1.2265; //sysid value for comp bot: 0.47892
                public static final double kV = isCompBot ? 0.025692 : 0.039238; // og 0.024236 0.035258;
                public static final double kA = isCompBot ? 0.0039338 : 0.012817; //og 0.0034545 //0.0031226; // 0.0053228;
                // public static final double kA = 0;
                // Game Object Heights
                public static final double CONE_HEIGHT = 6;
                public static final double CUBE_HIEGHT = 8;

                public static final int SENSOR_PORT = 0;
                public static final double KDt = 0.01;

                // constraints
                public static final double MAX_VELOCITY = 6; //6 // 10//50;
                public static final double MAX_ACCELERATION = 2; //2 //2 //25;

                // initial elevator stages
                public static final ElevatorState TELEOP_INIT_STATE = ElevatorState.ZEROED;

                public static final double ERROR = 50;

                public static final double GEAR_RATIO = isCompBot ? 5: 3;
                public static final double INCHES_PER_SPROCKET_ROTATION = 16.5;

                public static final double MASS = 8;
                public static final double BOTTOM = 0;
                public static final double MIDDLE = 10;
                public static final double TOP = 15;

                public static final double PULLEY_RADIUS = 2;

        }


        public static class VisionConstants {
                public static final String kCamera1Name = "beholder";
                public static final String kCamera2Name = "beholder";

                public static final double minDistFromTag = 0.3; // Min dist necessary from tag to automate (0.3 meter
                                                                 // aprox)
                public static final double xTolerance = 0.05;
                public static final double yTolerance = 0.03;
                public static final double thetaTolerance = 0.05;
                // Camera position on robot
                public static final Transform3d cam1ToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(9), 0), new Rotation3d(0, Units.degreesToRadians(-15), 0));// new Transform3d(new
                public static final Transform3d cam2ToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(9), 0), new Rotation3d(0, Units.degreesToRadians(-15), 0));// new Transform3d(new                

                public static final Map<Integer, Pose2d> kRedHPLocs = Map.of(
                        1,
                        new Pose2d(
                                        Units.inchesToMeters(602),
                                        Units.inchesToMeters(287),
                                        new Rotation2d(Math.PI)),
                        2,
                        new Pose2d(
                                        Units.inchesToMeters(602),
                                        Units.inchesToMeters(242),
                                        new Rotation2d(Math.PI)));

                public static final Map<Integer, Pose2d> kBlueHPLocs = Map.of(
                        1,
                        new Pose2d(
                                        Units.inchesToMeters(53),
                                        Units.inchesToMeters(287),
                                        new Rotation2d(0)),
                        2,
                        new Pose2d(
                                        Units.inchesToMeters(53),
                                        Units.inchesToMeters(242),
                                        new Rotation2d(0)));


                private static double redScoringOffsetsInches = -30; //initial values -30
                private static double blueScoringOffsetInches =  30;//initial values 30

                /**
                 * Key:
                 * Orientation: facing red community from blue community
                 * https://cdn.discordapp.com/attachments/453058111893405727/1062210473900126238/Screen_Shot_2023-01-09_at_7.25.00_PM.png
                 * Red Alliance Scoring Locations (right to left) – IDs 1, 2, ...9
                 * Blue Alliance Scoring Locations (left to right) – IDs 10, 11, ...18
                 */
                public static final Map<Integer, Pose2d> kRedScoreCols = Map.of(
                        1, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(610.69999999999999999999999 + redScoringOffsetsInches),
                                        Units.inchesToMeters(20.19),
                                        new Rotation2d(0)),
                        2, // Cube - 1
                        new Pose2d(
                                        Units.inchesToMeters(610.69999999999999999999999 + redScoringOffsetsInches),
                                        Units.inchesToMeters(42.19),
                                        new Rotation2d(0)),
                        3, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(610.69999999999999999999999 + redScoringOffsetsInches), // 597.1
                                        Units.inchesToMeters(64.19),
                                        new Rotation2d(0)),
                        4, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(610.69999999999999999999999 + redScoringOffsetsInches),
                                        Units.inchesToMeters(86.19),
                                        new Rotation2d(0)),
                        5, // Cube - 2
                        new Pose2d(
                                        Units.inchesToMeters(610.699999999999999999999998 + redScoringOffsetsInches), // 597.1
                                        Units.inchesToMeters(108.19),
                                        new Rotation2d(0)),
                        6, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(610.699999999999999999999998 + redScoringOffsetsInches),
                                        Units.inchesToMeters(130.19),
                                        new Rotation2d(0)),
                        7, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(610.699999999999999999999998 + redScoringOffsetsInches),
                                        Units.inchesToMeters(152.19),
                                        new Rotation2d(0)),
                        8, // Cube - 3
                        new Pose2d(
                                        Units.inchesToMeters(610.699999999999999999999998 + redScoringOffsetsInches),
                                        Units.inchesToMeters(174.19),
                                        new Rotation2d(0)),
                        9, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(610.699999999999999999999998 + redScoringOffsetsInches),
                                        Units.inchesToMeters(196.19),
                                        new Rotation2d(0)));

        
              //  scoringOffset = 0;
                public static final Map<Integer, Pose2d> kBlueScoreCols = Map.of(
                        9, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(196.19),
                                        new Rotation2d(Math.PI)),
                        8, // Cube - 6
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(174.19),
                                        new Rotation2d(Math.PI)),
                        7, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(152.19),
                                        new Rotation2d(Math.PI)),
                        6, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(130.19),
                                        new Rotation2d(Math.PI)),
                        5, // Cube - 7
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(108.19),
                                        new Rotation2d(Math.PI)),
                        4, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(86.19),
                                        new Rotation2d(Math.PI)),
                        3, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(64.19),
                                        new Rotation2d(Math.PI)),
                        2, // Cube - 8
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(42.19),
                                        new Rotation2d(Math.PI)),
                        1, // Cone
                        new Pose2d(
                                        Units.inchesToMeters(40.45 + blueScoringOffsetInches),
                                        Units.inchesToMeters(20.18888889),
                                        new Rotation2d(Math.PI)));

                /**
                 * Key:
                 * Orientation: Facing red community from blue community
                 * https://cdn.discordapp.com/attachments/453058111893405727/1062210473900126238/Screen_Shot_2023-01-09_at_7.25.00_PM.png
                 * Red Alliance Community (right to left) – IDs 1, 2, 3
                 * Blue Alliance Double Substation – ID 4
                 * Red Alliance Double Substation – ID 5
                 * Blue Alliance Community (left to right) – IDs 6, 7, 8
                 */
                public static final Map<Integer, Pose3d> aprilTags = Map.of(
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
}
