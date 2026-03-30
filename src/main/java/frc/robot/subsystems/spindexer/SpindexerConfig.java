package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SpindexerConfig {
  public static final SparkFlexConfig leaderConfig = new SparkFlexConfig();
  public static final SparkFlexConfig followerConfig = new SparkFlexConfig();

  static {
    leaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit, currentLimit);
    followerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit, currentLimit);
  }
}
