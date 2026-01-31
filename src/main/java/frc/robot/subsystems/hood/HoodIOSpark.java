package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class HoodIOSpark implements HoodIO{
    private final SparkMax leaderMotor = new SparkMax(HoodConstants.canId, MotorType.kBrushless);

    private final ConfigurableProfiledPIDController hoodController = new ConfigurableProfiledPIDController(0, 0, 0, null, "Hood");

    private RelativeEncoder hoodEncoder = leaderMotor.getEncoder();

    public HoodIOSpark() {
        
    }

    public void updateInputs(HoodIOInputs inputs) {
        double currentAngleDeg = Math.toDegrees(hoodEncoder.getPosition());
        inputs.relEncoder = currentAngleDeg;
    }

    @Override
    public void setAngle(double angle) {
        leaderMotor.set(MathUtil.clamp(hoodController.calculate(hoodEncoder.getPosition(), angle), -1, 1));
    }
}
