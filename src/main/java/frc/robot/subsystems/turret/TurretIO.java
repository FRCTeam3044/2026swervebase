package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public Angle targetAngle = Degrees.of(0.0);
    public Angle crtAngle = Degrees.of(0.0);
    public Angle angle = Degrees.of(0.0);
    public Angle driveAbsEncoderOne = Degrees.of(0.0);
    public Angle secondaryAbsEncoder = Degrees.of(0.0);
    public double driveRelEncoder = 0.0;
  }

  public default void setAngle(double angle) {}

  public default void updateInputs(TurretIOInputsAutoLogged inputs) {}

  public default void resetAngle() {}
}
