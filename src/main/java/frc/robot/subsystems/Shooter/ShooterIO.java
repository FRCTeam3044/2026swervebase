package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {}

  public default void setSpeed(double speed) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
  ;
}
