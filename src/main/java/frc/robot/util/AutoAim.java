package frc.robot.util;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShotCalculator.ShootingParameters;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AutoAim {
  private final Turret turret;
  private final Shooter shooter;
  private final Hood hood;

  private ShootingParameters parameters = new ShootingParameters(true, new Rotation2d(), 0, 0);

  private ConfigurableParameter<Double> shooterDisengagedProportion = new ConfigurableParameter<>(0.5,
      "ShooterDisengagedProportion");

  public AutoAim(Turret turret, Shooter shooter, Hood hood) {
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
  }

  public void periodic() {
    ShotCalculator.getInstance().clearShootingParameters();
    parameters = ShotCalculator.getInstance().getParameters();
  }

  public Command autoAimCmd(BooleanSupplier shooterEngaged) {
    return Commands.parallel(
        turret.setAngle(() -> parameters.turretAngle().getMeasure()),
        hood.setPosition(() -> parameters.hoodPosition()),
        shooter.runSpeed(
            () -> RPM.of(
                parameters.flywheelSpeed() * (shooterEngaged.getAsBoolean() ? 1 : shooterDisengagedProportion.get()))))
        .withName("Auto Aim");
  }
}
