package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;
  }
  
  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }
}