package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.hood.HoodConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

public class HoodIOSpark implements HoodIO {
  private final SparkMax motor = new SparkMax(canId, MotorType.kBrushless);

  private final ConfigurablePIDController hoodController = new ConfigurablePIDController(
      0.0, 0.1, 0.0, "Hood Controller");
  private final ConfigurableParameter<Double> stallCurrent = new ConfigurableParameter<Double>(0.25,
      "Hood Stall Current");
  private final ConfigurableParameter<Double> stallVelocityTolerance = new ConfigurableParameter<Double>(0.5,
      "Hood Stall Velocity Tolerance");
  private final ConfigurableParameter<Double> maxOutput = new ConfigurableParameter<>(10.0, "Hood Max Output");
  private final ConfigurableParameter<Double> minOutput = new ConfigurableParameter<>(10.0, "Hood Min Output");
  private final ConfigurableParameter<Double> errorFFTolerance = new ConfigurableParameter<>(1.5,
      "Hood Error FF Tolerance");
  private final ConfigurableParameter<Double> velocityScale = new ConfigurableParameter<>(5.0,
      "Hood Velocity Scale");
  private final ConfigurableParameter<Double> maxVelocity = new ConfigurableParameter<>(15.0, "Hood Max Velocity");
  private final ConfigurableParameter<Double> pidMax = new ConfigurableParameter<>(1.0, "Hood Max PID");

  private final RelativeEncoder hoodEncoder = motor.getEncoder();
  private double setpoint;
  LinearFilter currentFilter = LinearFilter.movingAverage(4);

  final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.001);
  public final ConfigurableParameter<Double> kV = new ConfigurableParameter<Double>(0.001, "Hood kV",
      feedforward::setKv);
  public final ConfigurableParameter<Double> kS = new ConfigurableParameter<Double>(0.0, "Hood kS",
      feedforward::setKs);
  public final ConfigurableParameter<Double> kPos = new ConfigurableParameter<Double>(0.0, "Hood kPos");

  private double error = 0.0;
  private double velocitySetpoint = 0.0;

  private boolean enabled = true;

  public HoodIOSpark() {
    tryUntilOk(
        motor,
        5,
        () -> motor.configure(
            HoodConfig.hoodConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(HoodIOInputs inputs) {
    ifOk(motor, hoodEncoder::getPosition, (value) -> inputs.position = value);
    ifOk(motor, hoodEncoder::getVelocity, (value) -> inputs.velocity = value);
    ifOk(motor, motor::getAppliedOutput, (value) -> inputs.output = value);
    ifOk(motor, motor::getOutputCurrent, (value) -> {
      inputs.current = Amps.of(value);
      inputs.currentAvg = Amps.of(currentFilter.calculate(value));
      inputs.stalled = inputs.currentAvg.in(Amps) > stallCurrent.get()
          && Math.abs(hoodEncoder.getVelocity()) < stallVelocityTolerance.get();
      // inputs.stalled = stallCurrent.get() < value;
    });
    inputs.setpoint = setpoint;
    inputs.error = error;
    inputs.velocitySetpoint = velocitySetpoint;
  }

  @Override
  public void setPosition(double position) {
    this.setpoint = position;

    double signedError = hoodEncoder.getPosition() - setpoint;
    this.error = signedError;
    double unsignedError = Math.abs(signedError);
    double clampedError = Math.max(unsignedError - errorFFTolerance.get(), 0);

    this.velocitySetpoint = -1 * Math.signum(signedError)
        * Math.min(clampedError * velocityScale.get(),
            maxVelocity.get());

    double ffOutput = feedforward.calculate(this.velocitySetpoint);

    double pidOutput = MathUtil.clamp(
        hoodController.calculate(hoodEncoder.getPosition(), this.setpoint),
        -pidMax.get(), pidMax.get());

    double posOutput = Math.signum(this.velocitySetpoint) * (maxPosition - hoodEncoder.getPosition()) * kPos.get();

    Logger.recordOutput("Hood/PidOutput", pidOutput);
    Logger.recordOutput("Hood/FFOutput", ffOutput);
    Logger.recordOutput("Hood/PosOutput", posOutput);
    Logger.recordOutput("Hood/TotalFFOutput", ffOutput + posOutput);

    double output = MathUtil.clamp(pidOutput + ffOutput + posOutput, -maxOutput.get(), maxOutput.get());

    if (!enabled) {
      motor.set(0);
      return;
    }

    if (Math.abs(output) < minOutput.get()) {
      motor.setVoltage(0);
    } else {
      motor.setVoltage(output);
    }
  }

  @Override
  public void setPercent(double percent) {
    if (!enabled) {
      motor.set(0);
      return;
    }
    motor.set(percent);
  }

  @Override
  public void resetPosition(double position) {
    hoodEncoder.setPosition(position);
  }

  @Override
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    if (!enabled) {
      motor.set(0);
    }
  }
}