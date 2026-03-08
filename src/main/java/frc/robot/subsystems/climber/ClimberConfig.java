package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimberConfig {
  public static SparkFlexConfig climberConfig = new SparkFlexConfig();

  static {
    SoftLimitConfig softLimits = new SoftLimitConfig();
    softLimits.forwardSoftLimit(100);
    softLimits.forwardSoftLimitEnabled(true);
    // softLimits.reverseSoftLimit(0);
    // softLimits.reverseSoftLimitEnabled(true);
    climberConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.currentLimit, ClimberConstants.currentLimit).inverted(true)
        .apply(softLimits);
  }
}
