// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    public static final int MOTOR_ID = 0;
    
    public static final int kPIDIdx = 0;
    public static final int P = 0;
    public static final int I = 0;
    public static final int D = 0;
    public static final int F = 0;

    // Wtvr it is
    public static final int METERS_PER_TICK = 0;
    public static final int INCHES_PER_TICK = 0;

    // Min/Max heights for the elevator (in inches)
    public static final double MAX_HEIGHT = 20;
    public static final double MIN_HEIGHT = 0;

    public static final double ZERO_HEIGHT = 0;
    public static final double SHELF_HEIGHT = 0;

    // MID, HIGH heights parwa cone
    public static final double MID_HEIGHT = 0;
    public static final double HIGH_HEIGHT = 0;

    // MID, HIGH heights para cube
    public static final double MID_HEIGHT = 0;
    public static final double HIGH_HEIGHT = 0;

    // TrapezoidProfile State constants
    public static final TrapezoidProfile.State ZERO_STATE = new TrapezoidProfile.State(Constants.Elevator.MID_HEIGHT, 0);
    public static final TrapezoidProfile.State MID_STATE = new TrapezoidProfile.State(Constants.Elevator.MID_HEIGHT, 0);
    public static final TrapezoidProfile.State HIGH_STATE = new TrapezoidProfile.State(Constants.Elevator.MID_HEIGHT, 0);
    public static final TrapezoidProfile.State SHEL_STATE = new TrapezoidProfile.State(Constants.Elevator.MID_HEIGHT, 0);

    // Game Object Heights
    public static final double CONE_HEIGHT = 6;
    public static final double CUBE_HIEGHT = 8;
  }
}
