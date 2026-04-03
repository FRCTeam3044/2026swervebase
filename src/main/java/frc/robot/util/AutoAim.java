package frc.robot.util;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShotCalculator.ShootingParameters;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AutoAim {
  private final Turret turret;
  private final Shooter shooter;
  private final Hood hood;
  private final AutoTargetUtil autoTargetUtil;
  private final Kicker kicker;
  private final Spindexer spindexer;

  private ShootingParameters parameters = new ShootingParameters(false, new Rotation2d(), 0, 0);

  private ConfigurableParameter<Double> shooterDisengagedProportion = new ConfigurableParameter<>(0.5,
      "ShooterDisengagedProportion");
  // private ConfigurableParameter<Double> hoodFudge = new
  // ConfigurableParameter<>(-0.5,
  // "HoodFudge");
  // private ConfigurableParameter<Double> shooterFudge = new
  // ConfigurableParameter<>(-300.0,
  // "ShooterFudge");

  public AutoAim(Turret turret, Shooter shooter, Hood hood, Kicker kicker, Spindexer spindexer,
      AutoTargetUtil autoTargetUtil) {
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.autoTargetUtil = autoTargetUtil;
  }

  public void periodic() {
    ShotCalculator.getInstance().clearShootingParameters();
    if (!autoTargetUtil.inAllianceZone()) {
      parameters = ShotCalculator.getInstance().getParameters(autoTargetUtil.getAllianceZoneTarget(), true);
    } else {
      parameters = ShotCalculator.getInstance().getParameters(autoTargetUtil.getHub(), false);
    }
  }

  public Command fire(BooleanSupplier forceFire) {
    return Commands.parallel(kicker.shootKicker(), spindexer.setSpeed())
        .onlyWhile(() -> (forceFire.getAsBoolean() || parameters.isValid()) && turret.isAtTarget() && hood.atPosition()
            && shooter.isAtSpeed())
        .repeatedly()
        .withName("Fire Shot");
  }

  public Command aimHub(BooleanSupplier shooterEngaged) {
    return Commands.parallel(
        turret.setAngle(() -> parameters.turretAngle().getMeasure()),
        hood.setPosition(() -> parameters.hoodPosition()/* + hoodFudge.get() */),
        shooter.runSpeed(
            () -> (/* shooterFudge.get() */ +parameters.flywheelSpeed())
                * (shooterEngaged.getAsBoolean() ? 1 : shooterDisengagedProportion.get())))
        .withName("Auto Aim at Hub");
  }

  public Command aimAllianceZone(BooleanSupplier shooterEngaged) {
    return Commands.parallel(
        turret.setAngle(() -> parameters.turretAngle().getMeasure()),
        hood.setPosition(() -> parameters.hoodPosition()),
        shooter.runSpeed(
            () -> parameters.flywheelSpeed() * (shooterEngaged.getAsBoolean() ? 1 : shooterDisengagedProportion.get())))
        .withName("Auto Aim at AZ");
  }
}
