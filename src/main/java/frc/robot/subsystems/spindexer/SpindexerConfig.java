package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SpindexerConfig {
  public static final SparkFlexConfig motorConfig = new com.revrobotics.spark.config.SparkFlexConfig();

  static {
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .inverted(true);
  }
}
