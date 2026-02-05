package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
  public static SparkMaxConfig motorConfigOne = new SparkMaxConfig();
  public static SparkMaxConfig motorConfigTwo = new SparkMaxConfig();

  static {
    motorConfigOne.idleMode(IdleMode.kCoast);
    motorConfigTwo.idleMode(IdleMode.kCoast);
  }
}
