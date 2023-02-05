package frc.robot.utils;

import frc.robot.Constants;

public class MathUtils {
    private static double INCHES_IN_A_METER = 39.3701;

    /**
     * Calculates the new input by the joystick after taking into account deadband
     * 
     * @param input raw input by the joystick
     * @param inputDeadband deadband for the value sent in
     * @return finalInput input by the joystick after calculating deadband
     */
    public static double handleDeadband(double input, double inputDeadband) {
        return (Math.abs(input) < inputDeadband)? 
            0.0 : 
            (input - inputDeadband * Math.signum(input)) / (1 - inputDeadband);
    }

    public static double rpmToTicks(double in_rpm, double gear_ratio) {
        return in_rpm / 600 * Constants.Talon.talonFXTicks * gear_ratio;
    }

    public static double ticksToRPM(double ticks, double ticks_per_rev, double gear_ratio) {
        return ticks * 600 / ticks_per_rev / gear_ratio;
    }

    public static int degreesToTicks(double degrees, double encoder_ticks, double gear_ratio) {
        return (int) ((encoder_ticks / gear_ratio) * degrees/360.0);
    }

    public static double ticksToDegrees(double ticks, double encoder_ticks, double gear_ratio) {
        return ticks / encoder_ticks * gear_ratio * 360.0;
    }

    public static int radiansToTicks(double radians, double encoder_ticks, double gear_ratio) {
        return (int) ((encoder_ticks / gear_ratio) * (radians/(2*Math.PI)));
    }

    public static double ticksToRadians(double ticks, double encoder_ticks, double gear_ratio) {
        return ticks / encoder_ticks * gear_ratio * 2 * Math.PI;
    }

    public static double mpsToRPM(double v_metersPerSecond, double radius) {
        return 60*v_metersPerSecond/(2 * Math.PI * radius);
    }

    public static double rpmToMPS(double v_RPM, double radius) {
        return (v_RPM / 60) * radius * 2 * Math.PI;
    }

    public static double ticksToMeter(double ticks, double encoder_ticks, double gear_ratio, double radius) {
        return (ticks/encoder_ticks) * gear_ratio * 2 * Math.PI * radius;
    }
    
    public static double inchesToMeters(double meters){
        return meters/INCHES_IN_A_METER;
    }

    /**
     * Check if a value is within a certain range of another
     * @param val
     * @param compare_val
     * @param epsilon
     * @return
     */
    public static boolean withinEpsilon(double val, double compare_val, double epsilon) {
        return (compare_val - epsilon) <= val && val <= (compare_val + epsilon); 
    }

    /**
     * returns if a value is within 0 and max (x in [0, max))
     * includes 0 and excludes max
     * @param val
     * @param max
     * @return boolean
     */
    public static boolean inRange(double val, double max) {
        return 0 <= val && val < max;
    }
}