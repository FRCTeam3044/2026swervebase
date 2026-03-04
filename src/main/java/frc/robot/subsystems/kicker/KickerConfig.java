package frc.robot.subsystems.kicker;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class KickerConfig {
  public static SparkFlexConfig topMotorConfig = new SparkFlexConfig();
  public static SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();

  static {
    topMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(KickerConstants.topCurrentLimit,
        KickerConstants.topCurrentLimit);
    bottomMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(KickerConstants.bottomCurrentLimit, KickerConstants.bottomCurrentLimit);
  }
}
