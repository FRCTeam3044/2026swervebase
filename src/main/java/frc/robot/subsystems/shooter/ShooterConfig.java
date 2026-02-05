package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterConfig {
  public static SparkFlexConfig motorConfig = new SparkFlexConfig();

  static {
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.currentLimit)
        .inverted(true);
  }
}
