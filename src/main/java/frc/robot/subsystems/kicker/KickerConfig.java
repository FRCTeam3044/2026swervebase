package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class KickerConfig {
  public static SparkFlexConfig motorConfigOne = new SparkFlexConfig();
  public static SparkFlexConfig motorConfigTwo = new SparkFlexConfig();

  static {
    motorConfigOne.idleMode(IdleMode.kCoast).smartCurrentLimit(KickerConstants.currentLimit);
    motorConfigTwo
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(KickerConstants.currentLimit)
        .follow(canIdOne);
  }
}
