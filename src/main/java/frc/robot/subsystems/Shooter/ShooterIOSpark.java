package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSpark implements ShooterIO {
  public final SparkFlex motor = new SparkFlex(ShooterConstants.kMotorPort, MotorType.kBrushless);

  public final SparkFlex motor2 = new SparkFlex(1, MotorType.kBrushless);

  private final RelativeEncoder encoder = motor.getEncoder();

  public ShooterIOSpark() {
    tryUntilOk(
        motor,
        5,
        () -> motor.configure(
            ShooterConfig.motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    tryUntilOk(
        motor2,
        5,
        () -> motor2.configure(
            ShooterConfig.motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
    motor2.set(-speed);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.current = motor.getOutputCurrent();
  }
}
