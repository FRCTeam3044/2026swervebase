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
  }

  public default void setSpeedIntake(double speed) {}

  public default void setSpeedRollers(double speed) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
  ;
}
