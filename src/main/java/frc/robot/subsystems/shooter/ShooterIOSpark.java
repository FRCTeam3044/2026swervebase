package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.shooter.ShooterConfig.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

public class ShooterIOSpark implements ShooterIO {
  public final SparkFlex leaderMotor = new SparkFlex(leaderCanId, MotorType.kBrushless);
  public final SparkFlex followerMotor = new SparkFlex(followerCanId, MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
  private final RelativeEncoder followerEncoder = followerMotor.getEncoder();
  private ConfigurablePIDController controller = new ConfigurablePIDController(0.0, 0.0, 0.0,
      "Shooter Speed Controller");

  private AngularVelocity targetSpeed = RPM.of(0);

  public ShooterIOSpark() {
    tryUntilOk(
        leaderMotor,
        5,
        () -> leaderMotor.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerMotor,
        5,
        () -> followerMotor.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // TODO: Feedforward and sysid?
  @Override
  public void setSpeed(AngularVelocity speed) {
    double leaderSetpoint = speed.in(RPM);
    this.targetSpeed = speed;
    leaderMotor.set(controller.calculate(leaderEncoder.getVelocity(), leaderSetpoint));
  }

  @Override
  public void setPercent(double percent) {
    leaderMotor.set(percent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> inputs.leaderVelocity = RPM.of(value));
    ifOk(followerMotor, followerEncoder::getVelocity, (value) -> inputs.followerVelocity = RPM.of(value));
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrent = value);
    ifOk(followerMotor, followerMotor::getOutputCurrent, (value) -> inputs.followerCurrent = value);
    inputs.targetSpeed = targetSpeed;
  }
}
