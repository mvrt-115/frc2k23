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

    /**
     * 
     * @param in_rpm
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @return ticks per 100 milliseconds
     */
    public static double rpmToTicks(double in_rpm, double gear_ratio) {
        return in_rpm / 600 * Constants.Talon.talonFXTicks * gear_ratio;
    }

    /**
     * 
     * @param ticks
     * @param ticks_per_rev # of ticks for one revolution of motor
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @return rpm
     */
    public static double ticksToRPM(double ticks, double ticks_per_rev, double gear_ratio) {
        return ticks * 600 / ticks_per_rev / gear_ratio;
    }

    /**
     * 
     * @param degrees
     * @param encoder_ticks
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @return
     */
    public static int degreesToTicks(double degrees, double encoder_ticks, double gear_ratio) {
        return (int) (encoder_ticks * gear_ratio * degrees/360.0);
    }

    /**
     * 
     * @param ticks
     * @param encoder_ticks
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @return
     */
    public static double ticksToDegrees(double ticks, double encoder_ticks, double gear_ratio) {
        return ticks / encoder_ticks / gear_ratio * 360.0;
    }

    /**
     * 
     * @param radians
     * @param encoder_ticks
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @return
     */
    public static int radiansToTicks(double radians, double encoder_ticks, double gear_ratio) {
        return (int) (encoder_ticks * gear_ratio * (radians/(2*Math.PI)));
    }

    /**
     * 
     * @param ticks
     * @param encoder_ticks
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @return
     */
    public static double ticksToRadians(double ticks, double encoder_ticks, double gear_ratio) {
        return ticks / encoder_ticks / gear_ratio * 2 * Math.PI;
    }

    /**
     * 
     * @param v_metersPerSecond
     * @param radius
     * @return
     */
    public static double mpsToRPM(double v_metersPerSecond, double radius) {
        return 60*v_metersPerSecond/(2 * Math.PI * radius);
    }

    /**
     * 
     * @param v_RPM
     * @param radius
     * @return
     */
    public static double rpmToMPS(double v_RPM, double radius) {
        return (v_RPM / 60) * radius * 2 * Math.PI;
    }

    /**
     * 
     * @param ticks
     * @param encoder_ticks
     * @param gear_ratio # of rotations of motor for one rotation of the manipulator
     * @param radius of wheel
     * @return
     */
    public static double ticksToMeter(double ticks, double encoder_ticks, double gear_ratio, double radius) {
        return (ticks/encoder_ticks) / gear_ratio * 2 * Math.PI * radius;
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

    public static double betterATanDeg(double x, double y) {
        if (x == 0) {
            if (y<0){
                return -90;
            }else{
                return 90;
            }
        }else{
            return Math.atan(y/x)*180/Math.PI;
        }
    }
}