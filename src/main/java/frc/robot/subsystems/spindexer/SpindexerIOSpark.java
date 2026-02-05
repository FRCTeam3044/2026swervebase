package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkFlex;
import java.util.function.DoubleSupplier;

public class SpindexerIOSpark implements SpindexerIO {
  private final SparkFlex motor = new SparkFlex(canId, MotorType.kBrushless);

  private RelativeEncoder encoderOne = motor.getEncoder();

  public SpindexerIOSpark() {
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                SpindexerConfig.motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motor, encoderOne::getVelocity, (value) -> inputs.speed = value);

    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
