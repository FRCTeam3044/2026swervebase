package frc.robot.subsystems.turret;

import frc.robot.RobotContainer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import me.nabdev.oxconfig.ConfigurableParameter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
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
  private final ConfigurableParameter<Double> wideTolerance = new ConfigurableParameter<Double>(8.0,
      "Turret Wide Angle Tolerance (degrees)");

  private final ConfigurableParameter<Double> turretRumbleTolerance = new ConfigurableParameter<>(15.0,
      "Turret Fliparound Tolerance");

  private final ConfigurableParameter<Double> slowSpeed = new ConfigurableParameter<>(1000.0, "Turret sSlow Speed");

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
    Logger.recordOutput("Turret/StopMoving", stopMoving);
    Logger.recordOutput("Turret/AtPosition", isAtTarget());
  }

  private boolean isOverridingTarget = false;
  private boolean stopMoving = false;

  public Command setAngle(Supplier<Angle> angle) {
    Supplier<Angle> realAngleSupplier = () -> {
      if (RobotContainer.getInstance().hood.calibrated()) {
        isOverridingTarget = false;
        stopMoving = false;
        return angle.get();
      }

      isOverridingTarget = true;

      if (!inHoodDangerZone()) {
        stopMoving = true;
        return inputs.angle;
      }

      stopMoving = false;

      double targetAngle = angle.get().in(Degrees);
      double dangerOneMax = hoodDangerOneMax.get() - 5;
      double dangerTwoMin = hoodDangerTwoMin.get() + 5;

      if (Math.abs(targetAngle - dangerOneMax) < Math.abs(targetAngle - dangerTwoMin)) {
        return Degrees.of(dangerOneMax + 5);
      } else {
        return Degrees.of(dangerTwoMin - 5);
      }
    };
    return Commands.runEnd(() -> {
      var tA = realAngleSupplier.get();
      if (stopMoving) {
        io.setPercent(0);
      } else {
        io.setAngle(tA);
      }
    }, () -> io.setPercent(0))
        .withName("Set Turret Angle");
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
    if (isOverridingTarget) {
      return false;
    }
    if (inputs.angle == null || inputs.computedTargetAngle == null) {
      return false;
    }
    return Math.abs(inputs.angle.in(Degrees) - inputs.computedTargetAngle.in(Degrees)) < tolerance.get();
  }

  public boolean isAtTargetWide() {
    if (isOverridingTarget) {
      return false;
    }
    if (inputs.angle == null || inputs.computedTargetAngle == null) {
      return false;
    }
    return Math.abs(inputs.angle.in(Degrees) - inputs.computedTargetAngle.in(Degrees)) < wideTolerance.get();
  }

  public boolean inHoodDangerZone() {
    return angleInHoodDangerZone(inputs.angle);
  }

  public boolean angleInHoodDangerZone(Angle angleToCheck) {
    double angle = angleToCheck.in(Degrees);
    return ((angle > hoodDangerOneMin.get() && angle < hoodDangerOneMax.get())
        || (angle > hoodDangerTwoMin.get() && angle < hoodDangerTwoMax.get()));
  }

  public boolean hasReset() {
    return inputs.hasReset;
  }

  public boolean nearFlipAround() {
    return distanceToFlipAround() < turretRumbleTolerance.get();
  }

  public double distanceToFlipAround() {
    double max = Math.abs(inputs.computedTargetAngle.minus(TurretConstants.maxAngle).in(Degrees));
    double min = Math.abs(inputs.computedTargetAngle.minus(TurretConstants.minAngle).in(Degrees));
    if (max < min) {
      return max;
    }
    return min;
  }

  public boolean movingSlow() {
    return Math.abs(inputs.angularVelocity.in(RPM)) < slowSpeed.get();
  }
}
