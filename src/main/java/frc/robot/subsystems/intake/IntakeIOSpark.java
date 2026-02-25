package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConfig.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax motorOne = new SparkMax(motorIdOne, MotorType.kBrushless);
  public final SparkMax motorTwo = new SparkMax(motorIdTwo, MotorType.kBrushless);

  private SparkMax rollerMotor = new SparkMax(rollerId, MotorType.kBrushless);

  private RelativeEncoder encoderOne = motorOne.getEncoder();
  private RelativeEncoder encoderTwo = motorTwo.getEncoder();

  private ConfigurablePIDController intakeController = new ConfigurablePIDController(0.0, 0.0, 0.0,
      "Intake Position Controller");

  private double targetPosition;

  public IntakeIOSpark() {
    tryUntilOk(
        motorOne,
        5,
        () -> motorOne.configure(
            motorConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        motorTwo,
        5,
        () -> motorTwo.configure(
            motorConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(rollerMotor, 5, () -> rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    ifOk(motorOne, motorOne::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motorOne, encoderOne::getVelocity, (value) -> inputs.speedIntake = value);

    ifOk(
        motorOne,
        new DoubleSupplier[] { motorOne::getAppliedOutput, motorOne::getBusVoltage },
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    ifOk(motorTwo, motorTwo::getOutputCurrent, (value) -> inputs.currentApms = value);
    ifOk(motorTwo, encoderTwo::getVelocity, (value) -> inputs.speedRollers = value);

    ifOk(
        motorTwo,
        new DoubleSupplier[] { motorTwo::getAppliedOutput, motorTwo::getBusVoltage },
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setIntakePosition(double position) {
    targetPosition = position;
    motorOne.set(intakeController.calculate(encoderOne.getPosition(), targetPosition));
  }

  @Override
  public void setIntakeSpeed(double speed) {
    motorOne.set(speed);
  }

  @Override
  public void setSpeedRollers(double speed) {
    rollerMotor.set(speed);
  }
}
