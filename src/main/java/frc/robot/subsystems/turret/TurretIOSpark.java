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

  private double currentAngleDeg;

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

  public void updateInputs(TurretIOInputsAutoLogged inputs) {
    ifOk(motor, driveRelEncoder::getPosition, (value) -> inputs.driveRelEncoder = value);
    ifOk(
        motor,
        driveAbsEncoder::getPosition,
        (value) -> inputs.driveAbsEncoderOne = Rotations.of(value));
    inputs.secondaryAbsEncoder = Rotations.of(secondaryAbsEncoder.get());
    inputs.crtAngle = Degrees.of(crt.getAngleOptional().get().in(Degrees));
  }

  @Override
  public void setAngle(double targetAngle) {
    motor.set(MathUtil.clamp(pidController.calculate(currentAngleDeg, targetAngle), -1.0, 1.0));
  }

  @Override
  public void resetAngle() {
    driveRelEncoder.setPosition(crt.getAngleOptional().get().in(Degrees));
  }
}
