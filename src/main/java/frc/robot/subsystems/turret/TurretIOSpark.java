package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import yams.units.EasyCRT.CRTStatus;

public class TurretIOSpark implements TurretIO {
  private final SparkFlex motor = new SparkFlex(canId, MotorType.kBrushless);

  private final RelativeEncoder driveRelEncoder = motor.getEncoder();
  private final DutyCycleEncoder driveAbsEncoder = new DutyCycleEncoder(primaryAbsEncoderDioChannel);
  private final DutyCycleEncoder secondaryAbsEncoder = new DutyCycleEncoder(secondaryAbsEncoderDioChannel);
  private final ConfigurableProfiledPIDController pidController = new ConfigurableProfiledPIDController(0.0, 0.1, 0.0,
      new Constraints(maxVelocity, maxAcceleration), "Turret PID");
  private final EasyCRT crt;

  private Angle currentAngle;
  private Angle rawTargetAngle;
  private Angle computedTargetAngle;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  private CRTStatus lastCRTStatus = null;
  private boolean crtMissing = false;
  private int crtIterations = 0;
  private double crtError = 0;
  private Angle lastCrtAngle = Degrees.of(0);

  public TurretIOSpark() {
    EasyCRTConfig crtConfig = new EasyCRTConfig(
        () -> Rotations.of(driveAbsEncoder.get()),
        () -> Rotations.of(secondaryAbsEncoder.get()))
        .withAbsoluteEncoder1Gearing(turretTeeth, primaryEncoderTeeth)
        .withAbsoluteEncoder2Gearing(turretTeeth, secondaryEncoderTeeth)
        .withAbsoluteEncoderOffsets(primaryAbsEncoderZero, secondaryAbsEncoderZero)
        .withAbsoluteEncoderInversions(true, true)
        .withMatchTolerance(Degrees.of(7))
        .withMechanismRange(minAngle, maxAngle);
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
    inputs.primaryAbsEncoder = driveAbsEncoder.get() * 2 * Math.PI;
    inputs.secondaryAbsEncoder = secondaryAbsEncoder.get() * 2 * Math.PI;
    inputs.rawTargetAngle = rawTargetAngle;
    inputs.computedTargetAngle = computedTargetAngle;
    inputs.angularVelocity = DegreesPerSecond.of(driveRelEncoder.getVelocity());
    inputs.targetAngularVelocity = DegreesPerSecond.of(pidController.getSetpoint().velocity);
    inputs.profileTargetPosition = Degrees.of(pidController.getSetpoint().position);
    inputs.crtStatus = lastCRTStatus;
    inputs.crtMissing = crtMissing;
    inputs.crtIterations = crtIterations;
    inputs.crtError = crtError;
    inputs.crtAngle = lastCrtAngle.in(Degrees);
    currentAngle = inputs.angle;
  }

  @Override
  public void setAngle(Angle targetAngle) {
    this.rawTargetAngle = targetAngle;

    // > 360 degrees (should probably handle better to allow using the extra range)
    // this.computedTargetAngle =
    Degrees.of(MathUtil.inputModulus(targetAngle.in(Degrees),
        minAngle.in(Degrees),
        maxAngle.in(Degrees)));
    // < 360 degrees
    this.computedTargetAngle = Degrees.of(MathUtil.clamp(targetAngle.in(Degrees), minAngle.in(Degrees),
        maxAngle.in(Degrees)));
    motor.set(pidController.calculate(currentAngle.in(Degrees), computedTargetAngle.in(Degrees))
        + feedforward.calculate(pidController.getSetpoint().velocity));
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts.in(Volts));
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  // private Angle getPrimaryAbsEncoderAngle() {
  // return Radians.of(MathUtil
  // .angleModulus(Rotations.of(driveAbsEncoder.get()).minus(primaryAbsEncoderZero)).in(Radians))
  // + Math.PI);
  // }

  // private Angle getSecondaryAbsEncoderAngle() {
  // return Radians.of(MathUtil.angleModulus(
  // Rotations.of(secondaryAbsEncoder.get()).minus(secondaryAbsEncoderZero)).in(Radians))
  // + Math.PI);
  // }

  @Override
  public void resetAngle() {
    Optional<Angle> crtAngle = crt.getAngleOptional();
    lastCRTStatus = crt.getLastStatus();
    crtMissing = crtAngle.isEmpty();
    crtIterations = crt.getLastIterations();
    crtError = Rotations.of(crt.getLastErrorRotations()).in(Degrees);
    if (crtAngle.isPresent()) {
      lastCrtAngle = Degrees.of(crtAngle.get().in(Degrees));
      driveRelEncoder.setPosition(crtAngle.get().in(Degrees));
    }
  }
}
