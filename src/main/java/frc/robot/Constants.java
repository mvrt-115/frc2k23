// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //robot as a whole constants
    public static final boolean debugMode = true;

    public static final double MAX_VOLTAGE = 10.0;
    
    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 35;
    public static final boolean kIsPracticeBot = true;
    public static final double kVoltageComp = 10.0;
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8); //numbers copied off of 2022 code; change to match this yr's bot
    
    public static class OperatorConstants {
       static final int kDriverControllerPort = 0;
    }

    public static class Intake {
        public static final double kCompressedTicks = 0; //MAKE THE ZERO POSITION THE COMPRESSED INTAKE PIECE POSITION
        public static final double kExpandedTicks = 50; // find the ticks when expanded at the most
        public static final int kGearRatio = 4; //temp
        public static final int kTicksPerRotation = 42; 
        public static final int kLengthIntake = 7;
        public static final int kMarginOfError = 20;

        public static final int kProximityPort = 0; //port number for element proximity sensor
        public static final int kProximityClawPort = 0; //port number for element proximity sensor

        public static final double kP = 0, kI = 0, kD = 0, kIz = 0 , kMaxOutput = 0, kMinOutput = 0;
    }
}
