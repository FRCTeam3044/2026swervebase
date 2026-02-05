package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.*;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import java.util.function.DoubleSupplier;

public class KickerIOSpark implements KickerIO {
  private final SparkFlex motorOne = new SparkFlex(canIdOne, MotorType.kBrushless); // vortex
  public final SparkFlex motorTwo = new SparkFlex(canIdTwo, MotorType.kBrushless);

  private RelativeEncoder encoderOne = motorOne.getEncoder();
  private RelativeEncoder encoderTwo = motorTwo.getEncoder();

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    ifOk(motorOne, motorOne::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motorOne, encoderOne::getVelocity, (value) -> inputs.speedMotorOne = value);

    ifOk(
        motorOne,
        new DoubleSupplier[] {motorOne::getAppliedOutput, motorOne::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    ifOk(motorTwo, motorTwo::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motorTwo, encoderTwo::getVelocity, (value) -> inputs.speedMotorTwo = value);

    ifOk(
        motorTwo,
        new DoubleSupplier[] {motorTwo::getAppliedOutput, motorTwo::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);
  }

  @Override
  public void setSpeed(double speedOne, double speedTwo) {
    motorOne.set(speedOne);
    motorTwo.set(speedTwo);
  }
}
