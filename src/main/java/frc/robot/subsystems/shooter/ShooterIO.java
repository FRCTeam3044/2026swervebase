package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double targetSpeed = 0.0;
    public AngularVelocity leaderVelocity = RPM.of(0);
    public AngularVelocity followerVelocity = RPM.of(0);
    public double calculatedGoal;
    public double leaderOutput;
    public double leaderCurrent = 0.0;
    public double followerCurrent = 0.0;
    public double ffOutput = 0.0;
    public double pidOutput = 0.0;
    public double pidError = 0.0;
    public AngularAcceleration targetAcceleration = RotationsPerSecondPerSecond.of(0);
  }

  public default void setSpeed(AngularVelocity speed) {
  }

  public default void setVoltage(Voltage volts) {
  }

  public default void setPercent(double percent) {
  }

  public default void updateInputs(ShooterIOInputs inputs) {
  };
}
