package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public AngularVelocity targetSpeed = RPM.of(0);
    public AngularVelocity leaderVelocity = RPM.of(0);
    public AngularVelocity followerVelocity = RPM.of(0);
    public double leaderCurrent = 0.0;
    public double followerCurrent = 0.0;
  }

  public default void setSpeed(AngularVelocity speed) {
  }

  public default void setPercent(double percent) {
  }

  public default void updateInputs(ShooterIOInputs inputs) {
  };
}
