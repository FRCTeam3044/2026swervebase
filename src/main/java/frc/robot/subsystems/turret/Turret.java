package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.turret.TurretConstants.maxAngle;
import static frc.robot.subsystems.turret.TurretConstants.minAngle;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Turret(TurretIO io) {
    this.io = io;

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Turret/SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setVoltage(voltage), null, this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> io.setVoltage(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction))
        .until(() -> {
          if (direction == SysIdRoutine.Direction.kForward) {
            return inputs.angle.gt(maxAngle);
          } else {
            return inputs.angle.lt(minAngle);
          }
        });
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setVoltage(Volts.of(0.0))).withTimeout(1.0).andThen(sysId.dynamic(direction)).until(() -> {
      if (direction == SysIdRoutine.Direction.kForward) {
        return inputs.angle.gt(maxAngle);
      } else {
        return inputs.angle.lt(minAngle);
      }
    });

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command setAngle(Supplier<Angle> angle) {
    return Commands.runEnd(() -> io.setAngle(angle.get()), () -> io.setPercent(0), this)
        .withName("Set Turret Angle");
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(() -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0), this)
        .withName("Run Turret At Percent");
  }

  public void resetAngle() {
    io.resetAngle();
  }

  public Angle getAngle() {
    return inputs.angle;
  }
}
