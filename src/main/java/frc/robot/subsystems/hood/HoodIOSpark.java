package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.hood.HoodConstants.stallCurrentLimit;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class HoodIOSpark implements HoodIO {
  private final SparkMax motor = new SparkMax(HoodConstants.canId, MotorType.kBrushless);

  private final ConfigurableProfiledPIDController hoodController = new ConfigurableProfiledPIDController(
      0.0, 0.1, 0.0, new Constraints(0, 0), "Hood Controller");
  private final ConfigurableParameter<Double> stallCurrentTolerance = new ConfigurableParameter<Double>(0.5,
      "Hood Stall Current Tolerance");
  private final ConfigurableParameter<Double> stallVelocityTolerance = new ConfigurableParameter<Double>(0.5,
      "Hood Stall Velocity Tolerance");

  private final RelativeEncoder hoodEncoder = motor.getEncoder();
  private double setpoint;

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
      inputs.stalled = Math.abs(value - stallCurrentLimit) < stallCurrentTolerance.get()
          && Math.abs(hoodEncoder.getVelocity()) < stallVelocityTolerance.get();
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
  public void resetPosition() {
    hoodEncoder.setPosition(0);
  }
}
