package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    public double testThing = 5;
  }
  
  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }
}