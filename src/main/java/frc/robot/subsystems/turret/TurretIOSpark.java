package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretIOSpark implements TurretIO {
  private final SparkFlex motor = new SparkFlex(canId, MotorType.kBrushless);

  private final RelativeEncoder driveRelEncoder = motor.getEncoder();
  private final AbsoluteEncoder driveAbsEncoder = motor.getAbsoluteEncoder();
  private final DutyCycleEncoder secondaryAbsEncoder = new DutyCycleEncoder(secondaryAbsEncoderDioChannel);
  private final ConfigurablePIDController pidController = new ConfigurablePIDController(0.0, 0.1, 0.0, "Turret");
  private final EasyCRT crt;

  private Angle currentAngle;
  private Angle rawTargetAngle;
  private Angle computedTargetAngle;

  public TurretIOSpark() {
    EasyCRTConfig crtConfig = new EasyCRTConfig(
        () -> Rotations.of(driveAbsEncoder.getPosition()),
        () -> Rotations.of(secondaryAbsEncoder.get()))
        .withCommonDriveGear(1, turretTeeth, primaryEncoderTeeth, secondaryEncoderTeeth)
        .withMatchTolerance(Degrees.of(0.1))
        .withMechanismRange(Degrees.of(0), Degrees.of(360));
    crt = new EasyCRT(crtConfig);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(
            TurretConfig.motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(TurretIOInputs inputs) {
    ifOk(motor, driveRelEncoder::getPosition, (value) -> inputs.angle = Degrees.of(value));
    ifOk(
        motor,
        driveAbsEncoder::getPosition,
        (value) -> inputs.driveAbsEncoderOne = Rotations.of(value));
    inputs.secondaryAbsEncoder = Rotations.of(secondaryAbsEncoder.get());
    inputs.crtAngle = Degrees.of(crt.getAngleOptional().get().in(Degrees));
    inputs.rawTargetAngle = rawTargetAngle;
    inputs.computedTargetAngle = computedTargetAngle;
    currentAngle = inputs.angle;
  }

  @Override
  public void setAngle(Angle targetAngle) {
    this.rawTargetAngle = targetAngle;

    // > 360 degrees (should probably handle better to allow using the extra range)
    // this.computedTargetAngle =
    // Degrees.of(MathUtil.inputModulus(targetAngle.in(Degrees),
    // minAngle.in(Degrees),
    // maxAngle.in(Degrees)));
    // < 360 degrees
    this.computedTargetAngle = Degrees.of(MathUtil.clamp(targetAngle.in(Degrees), minAngle.in(Degrees),
        maxAngle.in(Degrees)));
    motor.set(
        MathUtil.clamp(
            pidController.calculate(currentAngle.in(Degrees), targetAngle.in(Degrees)), -1.0, 1.0));
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void resetAngle() {
    driveRelEncoder.setPosition(crt.getAngleOptional().get().in(Degrees));
  }
}
