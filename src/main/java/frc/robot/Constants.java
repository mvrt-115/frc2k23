// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class JoystickControls {
    public static final boolean xBoxControl = false;
  }

  public static class SwerveDrivetrain {
    // Physical Constants
    public static final double chassisWidth = Units.inchesToMeters(26);
    public static final double chassisHeight = Units.inchesToMeters(28);

    // Important locations for swerve
    // consider swapping corners
    public static final Translation2d m_standardCenterLocation = new Translation2d(0, 0);
    public static final Translation2d m_frontLeftLocation = new Translation2d(-chassisWidth / 2.0, chassisHeight / 2.0);
    public static final Translation2d m_frontRightLocation = new Translation2d(chassisWidth / 2.0, chassisHeight / 2.0);
    public static final Translation2d m_backLeftLocation = new Translation2d(-chassisWidth / 2.0, -chassisHeight / 2.0);
    public static final Translation2d m_backRightLocation = new Translation2d(chassisWidth / 2.0, -chassisHeight / 2.0);
    public static final Translation2d[] rotatePoints = {
        m_standardCenterLocation, 
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
    };

    // Motor ID
    public static final int m_frontLeftDriveID = 3;
    public static final int m_frontRightDriveID = 1;
    public static final int m_backLeftDriveID = 5;
    public static final int m_backRightDriveID = 7;

    public static final int m_frontLeftTurnID = 4;
    public static final int m_frontRightTurnID = 2;
    public static final int m_backLeftTurnID = 6;
    public static final int m_backRightTurnID = 8;

    // Abs Encoder ID
    public static final int m_frontLeftEncoderID = 10;
    public static final int m_frontRightEncoderID = 9;
    public static final int m_backLeftEncoderID = 11;
    public static final int m_backRightEncoderID = 12;

    // Abs Encoder Offsets
    public static final double m_frontLeftEncoderOffset = Units.degreesToRadians(327);// + Math.PI/2.0;
    public static final double m_frontRightEncoderOffset = Units.degreesToRadians(163);// + Math.PI/2.0;
    public static final double m_backLeftEncoderOffset = Units.degreesToRadians(2);// + Math.PI/2.0;
    public static final double m_backRightEncoderOffset = Units.degreesToRadians(211);// + Math.PI/2.0;

    // Position PID
    public static final double m_x_control_P = 0.5;
    public static final double m_x_control_I = 0;
    public static final double m_x_control_D = 0;
    public static final double m_y_control_P = 0.5;
    public static final double m_y_control_I = 0;
    public static final double m_y_control_D = 0;
    public static final double m_r_control_P = 1;
    public static final double m_r_control_I = 0;
    public static final double m_r_control_D = 0;

    // Gyro CAN ID
    public static final int pigeon_id = 1;

    // constants for joystick drive
    public static final double kSensitivity = 0.90;
    public static final double kWheelDeadband = 0.2;
    public static final double kThrottleDeadband = 0.2;
    public static final double kWheelGain = 0.05;
    public static final double kWheelNonlinearity = 0.05;
    public static final double kMaxSpeedMPS = 5; // optimize max speed to prioritize translation
    public static final double kDriveMaxAcceleration = 2;
    public static final double kTurnMaxAcceleration = 2 * Math.PI;
    public static final double kDriveMaxSpeedMPS = 5;
    public static final double kTurnMaxSpeedRPS = 2 * Math.PI;
    public static final int kDriveJoystickPort = 0;
    public static final int kDriveXAxis = 0;
    public static final int kDriveYAxis = 1;
    public static final int kDriveWAxis = 4;
    public static final int kDriveFieldOrientButtonIdx = 1;

    // values to be determined after the robot is characterized
    public static final double kS = 0; // 0.69382 //units: Volts
    public static final double kV = 0; // 1.30485 //2.6097 //units: Volts * Seconds / Meters
    public static final double kA = 0; // 0.35228 //units: Volts * Seconds^2 / Meters

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kTurnMaxSpeedRPS, kTurnMaxAcceleration);
  }

  public static class Talon {
    public static final int talonFXTicks = 2048;
    public static final int talonSRXTicks = 4096;

    public static final double MAX_VOLTAGE = 10.0;

    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
    public static final boolean kIsPracticeBot = false;
    public static final double kVoltageComp = 10.0;
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40,
        50, 3.8);
  }

  public static class SwerveModule {
    public static final double gear_ratio_turn = 1.0 / 16.0 / 1.3;
    public static final double gear_ratio_drive = 6.75 / 1.0;
    public static final double radius = 0.05; // meters
    public static final double kwheelCircumference = 2 * Math.PI * radius; // meters

    // PID Constants
    public static final double kP = 0.2; // 3.2364;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    public static final double kPTurn = 0.1;
    public static final double kITurn = 0.0;
    public static final double kDTurn = 0.0;
    public static final double kFTurn = 0.0;
  }
}
