// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

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

    public static class VisionConstants{
        public static final String kCameraName = "sheeshcam";
        public static final Map<Integer, Pose3d> aprilTags = //relative to corner closest to tag 8
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
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
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
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
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