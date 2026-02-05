package frc.robot.subsystems.intake;

import static frc.robot.subsystems.kicker.KickerConstants.canIdOne;
import static frc.robot.subsystems.kicker.KickerConstants.canIdTwo;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax motorOne = new SparkMax(canIdOne, MotorType.kBrushless);
  public final SparkMax motorTwo = new SparkMax(canIdTwo, MotorType.kBrushless);

  private RelativeEncoder encoderOne = motorOne.getEncoder();
  private RelativeEncoder encoderTwo = motorTwo.getEncoder();

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ifOk(motorOne, motorOne::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motorOne, encoderOne::getVelocity, (value) -> inputs.speedIntake = value);

    ifOk(
        motorOne,
        new DoubleSupplier[] {motorOne::getAppliedOutput, motorOne::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    ifOk(motorTwo, motorTwo::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motorTwo, encoderTwo::getVelocity, (value) -> inputs.speedRollers = value);

    ifOk(
        motorTwo,
        new DoubleSupplier[] {motorTwo::getAppliedOutput, motorTwo::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);
  }

  @Override
  public void setSpeedIntake(double speed) {
    motorOne.set(speed);
  }

  @Override
  public void setSpeedRollers(double speed) {
    motorTwo.set(speed);
  }
}
