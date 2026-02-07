package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import static edu.wpi.first.units.Units.Degrees;
// import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class TurretConfig {
  public static SparkMaxConfig motorConfig = new SparkMaxConfig();

  static {
    EncoderConfig encoderConfig = new EncoderConfig();
    // Enable after we find the max mechanism range
    // encoderConfig.positionConversionFactor((maxAngle.in(Degrees) -
    // minAngle.in(Degrees)) / (maxPosition - minPosition));

    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.currentLimit)
        .apply(encoderConfig)
        .inverted(true);
  }
}
