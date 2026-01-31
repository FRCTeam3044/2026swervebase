package frc.robot.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class TurretIOSpark implements TurretIO{
    SparkMax motor = new SparkMax(0, MotorType.kBrushless);  

    private RelativeEncoder encoder = motor.getEncoder();
    
    private final ConfigurablePIDController pidController = new ConfigurablePIDController(0.0, 0.1, 0.0, "Turret");

    @Override
    public void setAngle(double speed) {
        motor.set(pidController.calculate(encoder.getPosition(), speed));
    }
}
