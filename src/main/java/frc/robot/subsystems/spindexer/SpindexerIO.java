package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double speed = 0.0;
    public double absEncoderOne = 0.0;
    public double absEncoderTwo = 0.0;
    public double relEncoder = 0.0;
  }

  public default void setSpeed(double speed) {
  }

  public default void updateInputs(SpindexerIOInputsAutoLogged inputs) {
  }
}
