package frc.robot.subsystems.spindexer;

import static frc.robot.util.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

public class SpindexerIOSpark implements SpindexerIO {
    private final SparkMax motor = new SparkMax(canId, MotorType.kBrushless);


  private RelativeEncoder encoderOne = motor.getEncoder();

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
