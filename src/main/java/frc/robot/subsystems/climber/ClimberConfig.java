package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimberConfig {
  public static SparkFlexConfig climberConfig = new SparkFlexConfig();

  static {
    climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ClimberConstants.currentLimit);
  }
}
