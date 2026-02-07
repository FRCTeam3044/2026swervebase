package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class HoodIOSpark implements HoodIO { // Outlines the HoodIOSpark class
  private final SparkMax motor = new SparkMax(HoodConstants.canId, MotorType.kBrushless); // Creates a new sparkmax
                                                                                          // motor and puts it into
                                                                                          // "motor"

  private final ConfigurableProfiledPIDController hoodController = new ConfigurableProfiledPIDController(
      0.0, 0.1, 0.0, new Constraints(0, 0), "Hood Controller");

  private final RelativeEncoder hoodEncoder = motor.getEncoder();
  private double setpoint;

  public HoodIOSpark() { // Attempts to connect with the Hood
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
}
