package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public Angle rawTargetAngle = Degrees.of(0.0);
    public Angle computedTargetAngle = Degrees.of(0.0);
    public Angle crtAngle = Degrees.of(0.0);
    public boolean crtMissing = false;
    public Angle angle = Degrees.of(0.0);
    public Angle primaryAbsEncoder = Degrees.of(0.0);
    public Angle secondaryAbsEncoder = Degrees.of(0.0);
    public AngularVelocity angularVelocity = DegreesPerSecond.of(0.0);
    public AngularVelocity targetAngularVelocity = DegreesPerSecond.of(0.0);
    public Angle profileTargetPosition = Degrees.of(0.0);
  }

  public default void setAngle(Angle angle) {
  }

  public default void setVoltage(Voltage volts) {

  }

  public default void setPercent(double percent) {
  }

  public default void updateInputs(TurretIOInputs inputs) {
  }

  public default void resetAngle() {
  }
}
