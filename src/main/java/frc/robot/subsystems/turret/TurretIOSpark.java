package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import static frc.robot.util.SparkUtil.tryUntilOk;


import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;


public class TurretIOSpark implements TurretIO{
    private final SparkMax motor = new SparkMax(TurretConstants.canId, MotorType.kBrushless);  

    private final RelativeEncoder encoder = motor.getEncoder();
    
    private final ConfigurablePIDController pidController = new ConfigurablePIDController(0.0, 0.1, 0.0, "Turret");

    private double currentAngleDeg;

    public TurretIOSpark() {
        tryUntilOk(motor, 5, () -> motor.configure(TurretConfig.motorConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));    
    }

    public void updateInputs(TurretIOInputsAutoLogged inputs) {
        currentAngleDeg = Math.toDegrees(encoder.getPosition());
        inputs.angle = currentAngleDeg;
    }

    @Override
    public void setAngle(double targetAngle) {
        motor.set(MathUtil.clamp(pidController.calculate(currentAngleDeg, targetAngle), -1.0, 1.0));
    }

    @Override
    public void resetAngle() {
        encoder.setPosition(0.0);
    }
}
