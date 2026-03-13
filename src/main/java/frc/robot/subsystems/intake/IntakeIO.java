package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double speedIntake = 0.0;
    public double speedRollers = 0.0;
    public double absEncoderOne = 0.0;
    public double absEncoderTwo = 0.0;
    public double relEncoder = 0.0;
    public double rollerCurrentApms = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double followerCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double targetPosition = 0.0;
  }

  public default void setIntakePosition(double angle) {
  }

  public default void setIntakeSpeed(double speed) {
  }

  public default void setSpeedRollers(double speed) {
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  };
}
