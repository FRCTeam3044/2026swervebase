package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import me.nabdev.oxconfig.ConfigurableParameter;

import static edu.wpi.first.units.Units.Degrees;
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

  private final ConfigurableParameter<Double> hoodDangerOneMin = new ConfigurableParameter<Double>(0.0,
      "Hood Danger One Min");
  private final ConfigurableParameter<Double> hoodDangerOneMax = new ConfigurableParameter<Double>(0.0,
      "Hood Danger One Max");
  private final ConfigurableParameter<Double> hoodDangerTwoMin = new ConfigurableParameter<Double>(0.0,
      "Hood Danger Two Min");
  private final ConfigurableParameter<Double> hoodDangerTwoMax = new ConfigurableParameter<Double>(0.0,
      "Hood Danger Two Max");

  private final ConfigurableParameter<Double> tolerance = new ConfigurableParameter<Double>(2.0,
      "Turret Angle Tolerance (degrees)");

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
            return inputs.angle.gt(maxAngle.minus(Degrees.of(10)));
          } else {
            return inputs.angle.lt(minAngle.plus(Degrees.of(10)));
          }
        });
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setVoltage(Volts.of(0.0))).withTimeout(1.0).andThen(sysId.dynamic(direction)).until(() -> {
      if (direction == SysIdRoutine.Direction.kForward) {
        return inputs.angle.gt(maxAngle.minus(Degrees.of(10)));
      } else {
        return inputs.angle.lt(minAngle.plus(Degrees.of(10)));
      }
    });

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/InHoodDangerZone", inHoodDangerZone());
  }

  public Command setAngle(Supplier<Angle> angle) {
    return Commands.runEnd(() -> io.setAngle(angle.get()), () -> io.setPercent(0), this)
        .withName("Set Turret Angle");
  }

  public Command exitDangerZone() {
    double curAngle = inputs.angle.in(Degrees);
    if (curAngle > hoodDangerOneMin.get() && curAngle < hoodDangerOneMax.get()) {
      return setAngle(() -> Degrees.of(hoodDangerOneMax.get() + 5))
          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    } else if (curAngle > hoodDangerTwoMin.get() && curAngle < hoodDangerTwoMax.get()) {
      return setAngle(() -> Degrees.of(hoodDangerTwoMin.get() - 5))
          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    } else {
      return Commands.none().withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(() -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0), this)
        .withName("Run Turret At Percent");
  }

  public void resetAngle(boolean reset) {
    io.resetAngle(reset);
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.angle.in(Degrees) - inputs.computedTargetAngle.in(Degrees)) < tolerance.get();
  }

  public boolean inHoodDangerZone() {
    double angle = inputs.angle.in(Degrees);
    return ((angle > hoodDangerOneMin.get() && angle < hoodDangerOneMax.get())
        || (angle > hoodDangerTwoMin.get() && angle < hoodDangerTwoMax.get()));
  }
}
