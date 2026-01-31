package frc.robot.subsystems.hood;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HoodConfig {
  public static SparkMaxConfig hoodConfig = new SparkMaxConfig();

  static {
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HoodConstants.currentLimit)
        .inverted(true);
  }
}
