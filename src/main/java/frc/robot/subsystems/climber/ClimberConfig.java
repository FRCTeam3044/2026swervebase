package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberConfig {
  public static SparkMaxConfig climberConfig = new SparkMaxConfig();

  static {
    climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ClimberConstants.currentLimit);
  }
}
