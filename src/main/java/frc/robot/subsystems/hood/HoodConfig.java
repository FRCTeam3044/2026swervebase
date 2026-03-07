package frc.robot.subsystems.hood;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodConfig {
  public static SparkMaxConfig hoodConfig = new SparkMaxConfig();

  static {
    SoftLimitConfig softLimits = new SoftLimitConfig();
    softLimits.forwardSoftLimit(maxPosition);
    softLimits.forwardSoftLimitEnabled(true);
    softLimits.reverseSoftLimit(minPosition);
    softLimits.reverseSoftLimitEnabled(true);

    hoodConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(stallCurrentLimit, freeCurrentLimit)
        .inverted(true)
        .apply(softLimits);
  }
}
