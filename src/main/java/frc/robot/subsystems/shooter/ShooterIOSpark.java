package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConfig.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class ShooterIOSpark implements ShooterIO {
  public final SparkFlex leaderMotor = new SparkFlex(leaderCanId, MotorType.kBrushless);
  public final SparkFlex followerMotor = new SparkFlex(followerCanId, MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
  private final RelativeEncoder followerEncoder = followerMotor.getEncoder();
  private ConfigurableProfiledPIDController controller = new ConfigurableProfiledPIDController(0.0, 0.0, 0.0,
      new Constraints(maxAcceleration, maxJerk), "Shooter Speed Controller");
  private final ConfigurableParameter<Double> bangBangThreshold = new ConfigurableParameter<>(200.0,
      "Bang Bang Threshold");
  private final ConfigurableParameter<Double> bangBangOutput = new ConfigurableParameter<>(0.5,
      "Bang Bang Output");

  private double targetSpeed = 0.0;
  private double calculatedGoal = 0.0;
  private double ffOutput = 0.0;
  private double pidOutput = 0.0;
  private double pidError = 0.0;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  public ShooterIOSpark() {
    controller.setIZone(1000);
    tryUntilOk(
        leaderMotor,
        5,
        () -> leaderMotor.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerMotor,
        5,
        () -> followerMotor.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setSpeed(double leaderSetpoint) {
    this.targetSpeed = leaderSetpoint;
    this.pidOutput = controller.calculate(leaderEncoder.getVelocity(), leaderSetpoint);
    this.pidError = controller.getPositionError();
    double goal = controller.getGoal().position;
    this.calculatedGoal = goal;
    this.ffOutput = feedforward.calculate(goal);
    double bbOut = 0;
    // if (targetSpeed - leaderEncoder.getVelocity() > bangBangThreshold.get()) {
    // bbOut = bangBangOutput.get();
    // }
    leaderMotor.setVoltage(this.pidOutput + this.ffOutput + bbOut);

  }

  @Override
  public void setVoltage(Voltage volts) {
    leaderMotor.setVoltage(volts.in(Volts));
  }

  @Override
  public void setPercent(double percent) {
    leaderMotor.set(percent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> inputs.leaderVelocity = value);
    ifOk(leaderMotor, leaderMotor::getAppliedOutput, (value) -> inputs.leaderOutput = value);
    ifOk(
        followerMotor,
        followerEncoder::getVelocity,
        (value) -> inputs.followerVelocity = value);
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrent = value);
    ifOk(followerMotor, followerMotor::getOutputCurrent, (value) -> inputs.followerCurrent = value);
    inputs.targetSpeed = targetSpeed;
    inputs.calculatedGoal = calculatedGoal;
    inputs.ffOutput = ffOutput;
    inputs.pidOutput = pidOutput;
    inputs.pidError = pidError;
  }
}
