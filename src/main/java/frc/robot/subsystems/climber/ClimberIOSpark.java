package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ClimberIOSpark implements ClimberIO {
  private final SparkFlex motor = new SparkFlex(ClimberConstants.canId, MotorType.kBrushless); // Fill in later

  private final RelativeEncoder climbEncoder = motor.getEncoder();
  private final DigitalInput limitSwitch = new DigitalInput(3);
  private final ConfigurableParameter<Double> runSpeedUp = new ConfigurableParameter<>(0.0,
      "Climber Motor Running Speed");
  // No current angle because it will be 0'd
  private final ConfigurableParameter<Double> runSpeedDown = new ConfigurableParameter<>(-0.0,
      "Climber Motor Running Speed");

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.bottomLimitPressed = limitSwitch.get();
    if (inputs.bottomLimitPressed) {
      climbEncoder.setPosition(0); // Reset encoder to 0 when bottom limit is pressed
    }
    inputs.currentPosition = climbEncoder.getPosition();
    ifOk(motor, climbEncoder::getPosition, (value) -> inputs.currentPosition = value);
  }

  // Skipping the HoodIOSpark connecting thing
  @Override
  public void setSpeed(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1));
  }

  @Override
  public void setClimberPos(double wantedHeight) {
    double currentPosition = climbEncoder.getPosition(); // Get current position
    if (wantedHeight > currentPosition) {
      motor.set(runSpeedUp.get());
    } else if (currentPosition > wantedHeight) {
      motor.set(runSpeedDown.get());
    } else {
      motor.set(0.0); // sets motor to 0
    }
  }
}
