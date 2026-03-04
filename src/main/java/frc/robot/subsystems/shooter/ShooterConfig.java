package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterConfig {
  public static SparkFlexConfig leaderConfig = new SparkFlexConfig();
  public static SparkFlexConfig followerConfig = new SparkFlexConfig();

  static {
    leaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit, currentLimit);

    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(currentLimit, currentLimit)
        .follow(leaderCanId, true);
  }
}
