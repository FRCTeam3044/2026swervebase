package frc.robot.subsystems.kicker;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerConfig {
  public static SparkMaxConfig motorConfig = new SparkMaxConfig();

  static {
    motorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(KickerConstants.currentLimit);
  }
}
