package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double speedIntake = 0.0;
    public double speedRollers = 0.0;
    public double absEncoderOne = 0.0;
    public double absEncoderTwo = 0.0;
    public double relEncoder = 0.0;
    public double angleDeg = 0.0;

    public double setpointRotations = 0.0;
    public double setpointRads = 0.0;
    public double hoodAngleRads = 0.0;
  }

  public default void setAngle(double angle) {}

  public default void updateInputs(HoodIOInputs inputs) {}
}
