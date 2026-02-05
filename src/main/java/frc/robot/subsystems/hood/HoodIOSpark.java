package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class HoodIOSpark implements HoodIO {
  private final SparkMax motor = new SparkMax(HoodConstants.canId, MotorType.kBrushless);

  private final ConfigurableProfiledPIDController hoodController =
      new ConfigurableProfiledPIDController(
          0.0, 0.1, 0.0, new Constraints(0, 0), "Hood Controller");

  private final RelativeEncoder hoodEncoder = motor.getEncoder();
  private double currentAngleDeg;

  public HoodIOSpark() {
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                HoodConfig.hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(HoodIOInputs inputs) {
    currentAngleDeg = Math.toDegrees(hoodEncoder.getPosition());
    inputs.angleDeg = currentAngleDeg;
  }

  @Override
  public void setAngle(double angle) {
    motor.set(MathUtil.clamp(hoodController.calculate(currentAngleDeg, angle), -1, 1));
  }
}
