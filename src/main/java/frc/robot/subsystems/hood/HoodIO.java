package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double position = 0.0;
    public double setpoint = 0.0;
  }

  public default void setPosition(double position) {}

  public default void setPercent(double percent) {}

  public default void updateInputs(HoodIOInputs inputs) {}
}
