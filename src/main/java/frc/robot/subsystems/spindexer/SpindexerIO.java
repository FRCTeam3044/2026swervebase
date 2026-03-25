package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double leaderSpeed = 0.0;
    public double followerSpeed = 0.0;
    public double leaderApms = 0.0;
    public double followerApms = 0.0;
    public double leaderOutput = 0.0;
    public double followerOutput = 0.0;
  }

  public default void setSpeed(double speed) {
  }

  public default void updateInputs(SpindexerIOInputs inputs) {
  }
}
