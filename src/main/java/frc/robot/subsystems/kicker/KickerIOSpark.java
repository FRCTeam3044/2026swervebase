package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConfig.*;
import static frc.robot.subsystems.kicker.KickerConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.function.DoubleSupplier;

public class KickerIOSpark implements KickerIO {
  private final SparkFlex topMotor = new SparkFlex(topMotorId, MotorType.kBrushless);
  public final SparkFlex bottomMotor = new SparkFlex(bottomMotorId, MotorType.kBrushless);

  private RelativeEncoder encoderOne = topMotor.getEncoder();
  private RelativeEncoder encoderTwo = bottomMotor.getEncoder();

  public KickerIOSpark() {
    tryUntilOk(
        topMotor,
        5,
        () -> topMotor.configure(
            motorConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        bottomMotor,
        5,
        () -> bottomMotor.configure(
            motorConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    ifOk(topMotor, topMotor::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(topMotor, encoderOne::getVelocity, (value) -> inputs.speedMotorOne = value);

    ifOk(
        topMotor,
        new DoubleSupplier[] { topMotor::getAppliedOutput, topMotor::getBusVoltage },
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    ifOk(bottomMotor, bottomMotor::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(bottomMotor, encoderTwo::getVelocity, (value) -> inputs.speedMotorTwo = value);

    ifOk(
        bottomMotor,
        new DoubleSupplier[] { bottomMotor::getAppliedOutput, bottomMotor::getBusVoltage },
        (values) -> inputs.appliedVoltage = values[0] * values[1]);
  }

  @Override
  public void setTopPercent(double percent) {
    topMotor.set(percent);
  }

  @Override
  public void setBottomPercent(double percent) {
    bottomMotor.set(percent);
  }
}
