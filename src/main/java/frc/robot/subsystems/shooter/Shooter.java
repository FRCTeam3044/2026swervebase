package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import me.nabdev.oxconfig.ConfigurableParameter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine sysId;
  private ConfigurableParameter<Double> tolerance = new ConfigurableParameter<Double>(5.0,
      "Shooter at speed tolerance (RPM)");

  public Shooter(ShooterIO io) {
    this.io = io;
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Shooter/SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setVoltage(voltage), null, this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> io.setVoltage(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setVoltage(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command runSpeed(DoubleSupplier speed) {
    return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setPercent(0.0), this)
        .withName("Run Shooter At Speed");
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(
        () -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0.0), this)
        .withName("Run Shooter At Percent");
  }

  public double getSpeed() {
    return inputs.leaderVelocity;
  }

  public boolean isAtSpeed(double targetSpeed) {
    return Math.abs(inputs.leaderVelocity - targetSpeed) < tolerance.get();
  }
}
