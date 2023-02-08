// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Elevator.ElevatorState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Elevator
  {
    public static final int MOTOR_ID = 13;
    public static final int MOTOR_ID2 = 14;
    
    public static final int kPIDIdx = 0;
    public static final double P = 0;//.01;
    public static final double I = 0;
    public static final double D = 0;
    public static final double F = 0;

    // Wtvr it is
    public static final double METERS_PER_TICK = .500;
    public static final double INCHES_PER_TICK = 10.6224;

    // Min/Max heights for the elevator (in inches)
    public static final double MAX_HEIGHT = 55;
    public static final double MIN_HEIGHT = 0;

    public static final double ZERO_HEIGHT = 0;
    public static final double SHELF_HEIGHT = 0;

    // MID, HIGH heights parwa cone (in inches)
    public static final double CONE_MID_HEIGHT = 34;
    public static final double CONE_HIGH_HEIGHT = 46;

    // MID, HIGH heights para cube (in inches)
    public static final double CUBE_MID_HEIGHT = 23.5;
    public static final double CUBE_HIGH_HEIGHT = 35.5;

    // feed forward constants
    public static final double kS = -0.55996;
    public static final double kG = 1.2265;
    public static final double kV = 0.035258;
   // public static final double kA = 0.0053228;
    public static final double kA = 0;
    // Game Object Heights
    public static final double CONE_HEIGHT = 6;
    public static final double CUBE_HIEGHT = 8;

    public static final int SENSOR_PORT = 0;
    public static final double KDt = 0.01;

    // constraints
    public static final double MAX_VELOCITY = 10.5;
    public static final double MAX_ACCELERATION = 6.5;

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
}
