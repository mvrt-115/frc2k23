package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class LevelPID
{
    private static final double time_diff = 0.02;
    
    public double P;
    public double I;
    public double D;
    public double FF;
    private double integral, derivative, error, prev_error, target = 0;

    public LevelPID(double P, double I , double D, double FF)
    {
        this.P = P;
        this.I = I;
        this.D = D;
        this.FF = FF;
    }
    
    /* Returns the swerve motor control value to level the robot with the platform
     * @param gyro the gyro that the robot is using to get angles
     * @return double the motor control value to fight the tipping platform
     */
    public double getLevelPIDSwerve(AHRS gyro)
    {
        // Theta = arctan(sqrt(tan^2(roll) + tan^2(pitch)))
        // Combines Pitch and Roll angles to get a combined vertical angle from the yaw axis

        double theta = gyro.getRoll()/Math.abs(gyro.getRoll())*Math.atan(Math.sqrt(tan_squared(Math.toRadians(gyro.getRoll()))+tan_squared(Math.toRadians(gyro.getPitch()))));

        // Target should be 0 most of the time because you are trying to level to a vertical angle of 0
        error = target - theta;
        integral = error*time_diff;
        derivative = (error-prev_error)/time_diff;
        prev_error = error;
        return P*error + I*this.integral + D*derivative;// + ff_function(theta); can ignore for now
    }

    /* Returns the WCD motor control value to level the robot with the platform
     * @param gyro the gyro that the robot is using to get angles
     * @return double the motor control value to fight the tipping platform
     */
    public double getLevelPIDWCD(AHRS gyro)
    {
        // Since this is a WCD, only the pitch is relevant
        double theta = gyro.getPitch();

        // Target should be 0 most of the time because you are trying to level to a vertical angle of 0
        error = target - theta;
        integral = error*time_diff;
        derivative = error/time_diff;
        return P*error + I*this.integral + D*derivative;// + ff_function(theta); can ignore for now
    }

    /* Feed Forward function for a backsliding system (more power when angle is farther from 0)
     * @param angle the vertical offset from the ground
     * @return double the feed forward value to put into the function
     */
    private double ff_function(double angle)
    {
        return FF*Math.sin(angle);
    }

    /* Returns tan^2(angle)
     * @param angle the angle that is entered into the function
     * @return double tangent squared of the inputted angle
     */
    private static double tan_squared(double angle)
    {
        return Math.pow(Math.tan(angle), 2);
    }
}
