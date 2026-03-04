package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
  public static SparkMaxConfig motorConfigOne = new SparkMaxConfig();
  public static SparkMaxConfig motorConfigTwo = new SparkMaxConfig();
  public static SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  static {
    motorConfigOne.idleMode(IdleMode.kCoast).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    motorConfigTwo.idleMode(IdleMode.kCoast).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit).follow(motorIdOne,
        true);
    rollerMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(rollerCurrentLimit, rollerCurrentLimit);
  }
}
