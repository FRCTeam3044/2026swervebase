package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConfig.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class ShooterIOSpark implements ShooterIO {
  public final SparkFlex leaderMotor = new SparkFlex(leaderCanId, MotorType.kBrushless);
  public final SparkFlex followerMotor = new SparkFlex(followerCanId, MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
  private final RelativeEncoder followerEncoder = followerMotor.getEncoder();
  private ConfigurableProfiledPIDController controller = new ConfigurableProfiledPIDController(0.0, 0.0, 0.0,
      new Constraints(maxVelocity, maxAcceleration), "Shooter Speed Controller");

  private AngularVelocity targetSpeed = RPM.of(0);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

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

  @Override
  public void setSpeed(AngularVelocity speed) {
    double leaderSetpoint = speed.in(RPM);
    this.targetSpeed = speed;
    leaderMotor
        .set(controller.calculate(leaderEncoder.getVelocity(), leaderSetpoint) + feedforward.calculate(leaderSetpoint));
  }

  @Override
  public void setVoltage(Voltage volts) {
    leaderMotor.setVoltage(volts.in(Volts));
  }

  @Override
  public void setPercent(double percent) {
    leaderMotor.set(percent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> inputs.leaderVelocity = RPM.of(value));
    ifOk(
        followerMotor,
        followerEncoder::getVelocity,
        (value) -> inputs.followerVelocity = RPM.of(value));
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrent = value);
    ifOk(followerMotor, followerMotor::getOutputCurrent, (value) -> inputs.followerCurrent = value);
    inputs.targetSpeed = targetSpeed;
  }
}
