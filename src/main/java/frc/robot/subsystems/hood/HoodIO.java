package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double position = 0.0;
    public double setpoint = 0.0;
    public double velocity = 0.0;
    public Current current = Amps.of(0.0);
    public Current currentAvg = Amps.of(0.0);
    public boolean stalled = false;
    public double error;
    public double velocitySetpoint;
    public double output;
  }

  public default void setPosition(double position) {
  }

  public default void setPercent(double percent) {
  }

  public default void resetPosition(double position) {
  }

  public default void updateInputs(HoodIOInputs inputs) {
  }
}
