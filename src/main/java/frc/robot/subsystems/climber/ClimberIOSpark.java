package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ClimberIOSpark implements ClimberIO {
  private final SparkFlex motor = new SparkFlex(ClimberConstants.canId, MotorType.kBrushless); // Fill in later

  private final RelativeEncoder climbEncoder = motor.getEncoder();
  private final DigitalInput limitSwitch = new DigitalInput(3);
  private final ConfigurableParameter<Double> runSpeedUp = new ConfigurableParameter<>(0.0,
      "Climber Motor Run Up Speed");
  private final ConfigurableParameter<Double> runSpeedDown = new ConfigurableParameter<>(-0.0,
      "Climber Motor Run Down Speed");
  private final ConfigurableParameter<Double> climberPosTolerance = new ConfigurableParameter<>(0.5,
      "Climber Position Tolerance");

  public ClimberIOSpark() {
    tryUntilOk(
        motor,
        5,
        () -> motor.configure(
            ClimberConfig.climberConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.bottomLimitPressed = limitSwitch.get();
    if (inputs.bottomLimitPressed) {
      climbEncoder.setPosition(0); // Reset encoder to 0 when bottom limit is pressed
    }
    inputs.currentPosition = climbEncoder.getPosition();
    ifOk(motor, climbEncoder::getPosition, (value) -> inputs.currentPosition = value);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.current = value);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(MathUtil.clamp(speed, limitSwitch.get() ? 0 : -1, 1));
  }

  @Override
  public void setClimberPos(double wantedHeight) {
    double currentPosition = climbEncoder.getPosition(); // Get current position
    if (Math.abs(currentPosition - wantedHeight) < climberPosTolerance.get()) {
      motor.set(0.0);
      return;
    }
    setSpeed(wantedHeight > currentPosition ? runSpeedUp.get() : runSpeedDown.get());
  }
}
