package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

  @AutoLog
  public static class KickerIOInputs {
    public double speedMotorOne = 0.0;
    public double speedMotorTwo = 0.0;
    public double absEncoderOne = 0.0;
    public double absEncoderTwo = 0.0;
    public double relEncoder = 0.0;
    public double currentApms = 0.0;
    public double appliedVoltage = 0.0;
  }

  public default void setSpeed(double speedOne, double speedTwo) {}

  public default void updateInputs(KickerIOInputs inputs) {}
}
