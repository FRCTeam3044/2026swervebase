package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class HoodIOSpark implements HoodIO {
  private final SparkMax motor = new SparkMax(HoodConstants.canId, MotorType.kBrushless);

  private final ConfigurableProfiledPIDController hoodController = new ConfigurableProfiledPIDController(
      0.0, 0.1, 0.0, new Constraints(0, 0), "Hood Controller");
  private final ConfigurableParameter<Double> stallCurrent = new ConfigurableParameter<Double>(0.25,
      "Hood Stall Current");
  private final ConfigurableParameter<Double> stallVelocityTolerance = new ConfigurableParameter<Double>(0.5,
      "Hood Stall Velocity Tolerance");

  private final RelativeEncoder hoodEncoder = motor.getEncoder();
  private double setpoint;
  LinearFilter currentFilter = LinearFilter.movingAverage(4);

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
    ifOk(motor, motor::getOutputCurrent, (value) -> {
      inputs.current = Amps.of(value);
      inputs.currentAvg = Amps.of(currentFilter.calculate(value));
      inputs.stalled = inputs.currentAvg.in(Amps) > stallCurrent.get()
          && Math.abs(hoodEncoder.getVelocity()) < stallVelocityTolerance.get();
      // inputs.stalled = stallCurrent.get() < value;
    });
    inputs.setpoint = setpoint;
  }

  @Override
  public void setPosition(double position) {
    setpoint = position;
    motor.set(hoodController.calculate(hoodEncoder.getPosition(), setpoint));
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void resetPosition(double position) {
    hoodEncoder.setPosition(position);
  }
}
