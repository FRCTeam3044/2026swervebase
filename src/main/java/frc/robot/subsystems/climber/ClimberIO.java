package frc.robot.subsystems.climber;

public interface ClimberIO {
  public static class ClimberIOInputs {
    public double currentPosition;
    public boolean bottomLimitPressed;
  }

  // Set speed;
  public default void setSpeed(double speed) {}

  public default void setClimberPos(double height) {}
}
