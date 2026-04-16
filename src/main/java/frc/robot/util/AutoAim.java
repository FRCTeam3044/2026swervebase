package frc.robot.util;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
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
  private final Drive drive;
  private boolean firing = false;

  private ShootingParameters parameters = new ShootingParameters(false, false, new Rotation2d(), 0, 0);

  private ConfigurableParameter<Double> shooterDisengagedProportion = new ConfigurableParameter<>(0.5,
      "ShooterDisengagedProportion");
  private ConfigurableParameter<Double> rotationalSpeedMax = new ConfigurableParameter<>(2.0,
      "Rotational Speed Max (rad/sec)");

  // private ConfigurableParameter<Double> hoodFudge = new
  // ConfigurableParameter<>(-0.5,
  // "HoodFudge");
  // private ConfigurableParameter<Double> shooterFudge = new
  // ConfigurableParameter<>(-300.0,
  // "ShooterFudge");

  public AutoAim(Turret turret, Shooter shooter, Hood hood, Kicker kicker, Spindexer spindexer,
      AutoTargetUtil autoTargetUtil, Drive drive) {
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.autoTargetUtil = autoTargetUtil;
    this.drive = drive;
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
    return fire(forceFire, () -> false, true);
  }

  public Command fire(BooleanSupplier forceFire, BooleanSupplier jiggleIndexer, boolean runKicker) {
    BooleanSupplier safeShoot = () -> {
      boolean force = forceFire.getAsBoolean();
      boolean hoodGood = hood.atPosition();
      boolean shooterGood = shooter.isAtSpeed();
      boolean turretGood = parameters.isTurretValid();
      boolean distanceGood = parameters.isDistanceValid();
      boolean otherSubsystemsAtPos = hoodGood && shooterGood;
      // boolean turretGood = autoTargetUtil.inAllianceZone() ? turret.isAtTarget() :
      // turret.isAtTargetWide();
      boolean angularVelocityGood = Math.abs(drive.getVelocity().omegaRadiansPerSecond) < rotationalSpeedMax.get();
      boolean firing = turretGood && (force || (otherSubsystemsAtPos && angularVelocityGood && distanceGood));

      Logger.recordOutput("AutoAim/ForcingFire", force);
      Logger.recordOutput("AutoAim/AngularVelocityGood", angularVelocityGood);
      // Logger.recordOutput("AutoAim/TurretGood", turretGood);
      Logger.recordOutput("AutoAim/HoodGood", hoodGood);
      Logger.recordOutput("AutoAim/ShooterGood", shooterGood);
      Logger.recordOutput("AutoAim/turretGood", turretGood);
      Logger.recordOutput("AutoAim/distanceGood", distanceGood);
      Logger.recordOutput("AutoAim/Firing", firing);

      return firing;
    };

    if (runKicker) {
      return Commands
          .parallel(kicker.shootKicker(), Commands.parallel(spindexer.setSpeed(jiggleIndexer).onlyWhile(safeShoot)
              .repeatedly(),
              Commands.runEnd(() -> firing = true, () -> firing = false)))

          .withName("Fire Shot");
    } else {
      return Commands
          .parallel(Commands.parallel(spindexer.setSpeed(jiggleIndexer), kicker.shootKicker()).onlyWhile(safeShoot)
              .repeatedly(),
              Commands.runEnd(() -> firing = true, () -> firing = false))
          .withName("Fire Shot");
    }
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

  public boolean firing() {
    Logger.recordOutput("Firing", firing);
    return firing;
  }
}
