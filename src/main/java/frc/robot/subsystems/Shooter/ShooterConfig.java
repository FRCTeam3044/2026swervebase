package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConfig {
    public static SparkFlexConfig motorConfig = new SparkFlexConfig();

    static {
        motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.currentLimit).inverted(true);
    }
}
