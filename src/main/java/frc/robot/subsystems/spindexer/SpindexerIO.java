package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double bottomSpeed = 0.0;
    public double topSpeed = 0.0;
    public double bottomApms = 0.0;
    public double topApms = 0.0;
    public double bottomOutput = 0.0;
    public double topOutput = 0.0;
  }

  public default void setTopSpeed(double speed) {
  }

  public default void setBottomSpeed(double speed) {
  }

  public default void updateInputs(SpindexerIOInputs inputs) {
  }
}
