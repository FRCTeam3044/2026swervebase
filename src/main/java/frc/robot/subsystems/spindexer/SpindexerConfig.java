package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SpindexerConfig {
    public static final SparkFlexConfig motorConfig =
        new com.revrobotics.spark.config.SparkFlexConfig();
    
    static {
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SpindexerConstants.currentLimit)
        .inverted(true);
  }
}
