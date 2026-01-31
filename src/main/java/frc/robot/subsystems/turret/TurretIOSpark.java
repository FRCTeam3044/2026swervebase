package frc.robot.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretIOSpark implements TurretIO{
    SparkMax motor = new SparkMax(0, MotorType.kBrushless);  

    private RelativeEncoder encoder = motor.getEncoder();

    @Override
    public void setAngle(double speed) {

    }
}
