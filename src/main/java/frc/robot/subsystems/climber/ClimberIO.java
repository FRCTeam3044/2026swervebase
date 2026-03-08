package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double currentPosition;
    public boolean bottomLimitPressed;
    public double current;
  }

  public default void updateInputs(ClimberIOInputs inputs) {

  }

  // Set speed;
  public default void setSpeed(double speed) {
  }

  public default void setClimberPos(double height) {
  }
}
