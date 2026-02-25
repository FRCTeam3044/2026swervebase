package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import static edu.wpi.first.units.Units.Degrees;
// import static frc.robot.subsystems.turret.TurretConstants.*;

public class TurretConfig {
  public static SparkMaxConfig motorConfig = new SparkMaxConfig();

  static {
    EncoderConfig encoderConfig = new EncoderConfig();
    SoftLimitConfig softLimits = new SoftLimitConfig();
    // Enable after we find the max mechanism range
    // double conversionFactor = (maxAngle.in(Degrees) - minAngle.in(Degrees)) /
    // (maxPosition - minPosition);
    // encoderConfig.positionConversionFactor(conversionFactor)
    // .velocityConversionFactor(conversionFactor);
    // softLimits.forwardSoftLimitEnabled(true);
    // softLimits.forwardSoftLimit(TurretConstants.maxPosition);
    // softLimits.reverseSoftLimitEnabled(true);
    // softLimits.reverseSoftLimit(TurretConstants.minPosition);

    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.currentLimit)
        .apply(softLimits)
        .apply(encoderConfig)
        .inverted(true);
  }
}
