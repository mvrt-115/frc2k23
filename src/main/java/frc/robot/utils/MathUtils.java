// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class MathUtils
{
    private static double INCHES_IN_A_METER = 39.3701;
    
    public static double inchesToMeters(double meters)
    {
        return meters/INCHES_IN_A_METER;
    }
}