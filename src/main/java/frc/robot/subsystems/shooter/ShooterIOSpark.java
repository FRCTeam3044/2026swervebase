package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterConfig.*;

public class ShooterIOSpark implements ShooterIO {
  public final SparkFlex leaderMotor = new SparkFlex(leaderCanId, MotorType.kBrushless);
  public final SparkFlex followerMotor = new SparkFlex(followerCanId, MotorType.kBrushless);

  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  public ShooterIOSpark() {
    tryUntilOk(
        leaderMotor,
        5,
        () -> leaderMotor.configure(
            leaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    tryUntilOk(
        followerMotor,
        5,
        () -> followerMotor.configure(
            followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  @Override
  public void setSpeed(double speed) {
    leaderMotor.set(speed);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.current = leaderMotor.getOutputCurrent();
  }
}