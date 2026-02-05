package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double angle = 0.0;
    public double absEncoderOne = 0.0;
    public double absEncoderTwo = 0.0;
    public double relEncoder = 0.0;
  }

  public default void setAngle(double angle) {}

  public default void updateInputs(TurretIOInputsAutoLogged inputs) {}

  public default void resetAngle() {}
}
