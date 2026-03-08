package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

public class TurretIOSpark implements TurretIO {
  private final SparkFlex motor = new SparkFlex(canId, MotorType.kBrushless);

  private final RelativeEncoder driveRelEncoder = motor.getEncoder();
  private final DutyCycleEncoder driveAbsEncoder = new DutyCycleEncoder(primaryAbsEncoderDioChannel);
  private final DutyCycleEncoder secondaryAbsEncoder = new DutyCycleEncoder(secondaryAbsEncoderDioChannel);
  private final ConfigurablePIDController pidController = new ConfigurablePIDController(0.0, 0.1, 0.0, "Turret PID");
  private final ConfigurableParameter<Double> maxOutput = new ConfigurableParameter<>(10.0, "Turret Max Output");
  private final ConfigurableParameter<Double> errorFFTolerance = new ConfigurableParameter<>(10.0,
      "Turret Error FF Tolerance");
  private final ConfigurableParameter<Double> velocityScale = new ConfigurableParameter<>(10.0,
      "Turret Velocity Scale");
  private final ConfigurableParameter<Double> maxVelocity = new ConfigurableParameter<>(10.0, "Turret Max Velocity");
  private final ConfigurableParameter<Double> pidMax = new ConfigurableParameter<>(1.0, "Turret Max PID");

  // private final EasyCRT crt;

  private Angle currentAngle;
  private Angle rawTargetAngle;
  private Angle computedTargetAngle;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  LinearFilter filterOne = LinearFilter.movingAverage(500);
  LinearFilter filterTwo = LinearFilter.movingAverage(500);

  // private CRTStatus lastCRTStatus = null;
  private boolean crtMissing = false;
  // private int crtIterations = 0;
  // private double crtError = 0;
  private Angle lastCrtAngle = Degrees.of(0);

  private double calibrationEncoderReading = Double.NaN;
  private double calibrationAngleDeg = Double.NaN;
  private double error = 0.0;
  private double velocitySetpoint = 0.0;

  private final double degreesPerEncoderUnit = (maxAngle.in(Degrees) - minAngle.in(Degrees))
      / (maxPosition - minPosition);

  public TurretIOSpark() {
    // EasyCRTConfig crtConfig = new EasyCRTConfig(
    // () -> Rotations.of(driveAbsEncoder.get()),
    // () -> Rotations.of(secondaryAbsEncoder.get()))
    // .withAbsoluteEncoder1Gearing(turretTeeth, primaryEncoderTeeth)
    // .withAbsoluteEncoder2Gearing(turretTeeth, secondaryEncoderTeeth)
    // .withAbsoluteEncoderOffsets(primaryAbsEncoderZero, secondaryAbsEncoderZero)
    // .withAbsoluteEncoderInversions(true, true)
    // .withMatchTolerance(Degrees.of(7))
    // .withMechanismRange(minAngle, maxAngle);
    // crt = new EasyCRT(crtConfig);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(
            TurretConfig.motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(TurretIOInputs inputs) {
    ifOk(motor, driveRelEncoder::getPosition, (value) -> inputs.rawPosition = value);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.current = value);
    ifOk(motor, motor::getAppliedOutput, (value) -> inputs.output = value);
    inputs.primaryAbsEncoder = getPrimaryAbsEncoderAngle();
    inputs.secondaryAbsEncoder = getSecondaryAbsEncoderAngle();
    inputs.rawPrimaryEncoderAvgDeg = filterOne.calculate(Rotations.of(driveAbsEncoder.get()).in(Degrees));
    inputs.rawSecondaryEncoderAvgDeg = filterTwo.calculate(Rotations.of(secondaryAbsEncoder.get()).in(Degrees));
    inputs.rawTargetAngle = rawTargetAngle;
    inputs.computedTargetAngle = computedTargetAngle;
    inputs.angularVelocity = DegreesPerSecond.of(driveRelEncoder.getVelocity());
    inputs.calibrationAngleDeg = calibrationAngleDeg;
    inputs.calibrationEncoderReading = calibrationEncoderReading;
    inputs.degreesPerEncoderUnit = degreesPerEncoderUnit;
    // inputs.crtStatus = lastCRTStatus;
    inputs.crtMissing = crtMissing;
    // inputs.crtIterations = crtIterations;
    // inputs.crtError = crtError;
    inputs.crtAngle = lastCrtAngle;
    if (!Double.isNaN(calibrationEncoderReading)) {
      inputs.angle = Degrees.of(getAngleFromRel(driveRelEncoder.getPosition()));
    }
    currentAngle = inputs.angle;
    inputs.error = error;
    inputs.velocitySetpoint = velocitySetpoint;
  }

  private double getAngleFromRel(double encoder) {
    return calibrationAngleDeg + (encoder - calibrationEncoderReading) * degreesPerEncoderUnit;
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
    // this.computedTargetAngle = Degrees.of(MathUtil.clamp(targetAngle.in(Degrees),
    // minAngle.in(Degrees),
    // maxAngle.in(Degrees)));

    if (targetAngle.gt(minAngle) && targetAngle.lt(maxAngle)) {
      computedTargetAngle = targetAngle;
    } else {
      double distToMin = MathUtil.inputModulus(targetAngle.in(Degrees) - minAngle.in(Degrees), 0, 360);
      double distToMax = MathUtil.inputModulus(targetAngle.in(Degrees) - maxAngle.in(Degrees), 0, 360);

      computedTargetAngle = distToMin > distToMax ? maxAngle : minAngle;
    }

    double pidOutput = MathUtil.clamp(
        pidController.calculate(currentAngle.in(Degrees), computedTargetAngle.in(Degrees)),
        -pidMax.get(), pidMax.get());

    double signedError = currentAngle.in(Degrees) - computedTargetAngle.in(Degrees);
    this.error = signedError;
    double unsignedError = Math.abs(signedError);

    this.velocitySetpoint = -1 * Math.signum(signedError)
        * Math.min(Math.max(unsignedError - errorFFTolerance.get(), 0) * velocityScale.get(),
            maxVelocity.get());

    double ffOutput = feedforward.calculate(this.velocitySetpoint);

    Logger.recordOutput("Turret/PidOutput", pidOutput);
    Logger.recordOutput("Turret/FFOutput", ffOutput);

    double output = MathUtil.clamp(pidOutput + ffOutput, -maxOutput.get(), maxOutput.get());
    motor.setVoltage(applySoftLimits(output));
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(applySoftLimits(volts.in(Volts)));
  }

  @Override
  public void setPercent(double percent) {
    motor.set(applySoftLimits(percent));
  }

  private Angle getPrimaryAbsEncoderAngle() {
    return Radians
        .of(MathUtil.angleModulus(Rotations.of(driveAbsEncoder.get()).minus(primaryAbsEncoderZero).in(Radians))
            + Math.PI);
  }

  private Angle getSecondaryAbsEncoderAngle() {
    return Radians.of(MathUtil.angleModulus(
        Rotations.of(secondaryAbsEncoder.get()).minus(secondaryAbsEncoderZero).in(Radians))
        + Math.PI);
  }

  @Override
  public void resetAngle(boolean reset) {
    Optional<Angle> crtAngle = getAngle();

    crtMissing = crtAngle.isEmpty();
    if (crtAngle.isPresent()) {
      Angle angle = crtAngle.get();
      lastCrtAngle = angle;
      if (reset) {
        calibrationEncoderReading = driveRelEncoder.getPosition();
        calibrationAngleDeg = angle.in(Degrees);
      }
    }
  }

  // public Optional<Angle> getAngle() {
  // Optional<Angle> crtAngle = crt.getAngleOptional();
  // lastCRTStatus = crt.getLastStatus();
  // crtIterations = crt.getLastIterations();
  // crtError = Rotations.of(crt.getLastErrorRotations()).in(Degrees);
  // return crtAngle;
  // }

  private Optional<Angle> getAngle() {
    long teeth1 = Math.round(getPrimaryAbsEncoderAngle().in(Rotations) * primaryEncoderTeeth);
    long teeth2 = Math.round(getSecondaryAbsEncoderAngle().in(Rotations) * secondaryEncoderTeeth);

    long lcm = (long) primaryEncoderTeeth * secondaryEncoderTeeth; // 594
    long turretGearTeeth = ((teeth1 * secondaryEncoderTeeth * 9L + teeth2 * primaryEncoderTeeth * 16L) % lcm + lcm)
        % lcm;

    Logger.recordOutput("Turret/TurretGearTeeth", turretGearTeeth);

    if (turretGearTeeth > turretTeeth * 1.2) {
      return Optional.empty();
    }

    return Optional.of(Rotations.of((double) turretGearTeeth / turretTeeth));
  }

  private double applySoftLimits(double num) {
    if (currentAngle.gt(maxAngle) && num > 0) {
      return 0;
    } else if (currentAngle.lt(minAngle) && num < 0) {
      return 0;
    }
    return num;
  }
}
