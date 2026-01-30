package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    
  @AutoLog
  public static class KickerIOInputs {
    public double speedIntake = 0.0;
    public double speedRollers = 0.0;
    public double absEncoderOne = 0.0;
    public double absEncoderTwo = 0.0;
    public double relEncoder = 0.0;
  }
  public default void setSpeed(double speed) {}
  public default void updateInputs(KickerIOInputs inputs) {}
}
