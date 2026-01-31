package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class TurretConfig {
    public static SparkMaxConfig motorConfig = new SparkMaxConfig();

    static {
        motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(TurretConstants.currentLimit).inverted(true);
    }
}
