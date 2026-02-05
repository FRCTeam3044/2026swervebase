package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

public class TurretIOSpark implements TurretIO {
  private final SparkFlex motor = new SparkFlex(TurretConstants.canId, MotorType.kBrushless); // vortex

  private final RelativeEncoder encoder = motor.getEncoder();

  private final ConfigurablePIDController pidController =
      new ConfigurablePIDController(0.0, 0.1, 0.0, "Turret");

  private double currentAngleDeg;

  public TurretIOSpark() {
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                TurretConfig.motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
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
