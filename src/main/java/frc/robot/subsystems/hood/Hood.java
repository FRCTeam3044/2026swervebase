package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final ConfigurableParameter<Double> calibrationSpeed = new ConfigurableParameter<Double>(-0.05,
      "Hood Calibration Speed");

  private final ConfigurableParameter<Double> hoodPositionTolerance = new ConfigurableParameter<>(3.0,
      "Hood Position Tolerance");

  public Hood(HoodIO io) {
    this.io = io;
  }

  private boolean calibrated = false;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command setPosition(DoubleSupplier position) {
    return calibrateIfNeeded(Commands.runEnd(
        () -> io.setPosition(position.getAsDouble()), () -> io.setPercent(0), this)).withName("Set Hood Position");
  }

  public Command runPercent(DoubleSupplier percent) {
    return calibrateIfNeeded(Commands.runEnd(() -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0), this))
        .withName("Run Hood At Percent");
  }

  private Command calibrate() {
    return Commands.runOnce(() -> io.resetPosition(200.0))
        .andThen(Commands.waitUntil(() -> !RobotContainer.getInstance().turret.inHoodDangerZone()))
        .andThen(Commands.run(() -> io.setPercent(calibrationSpeed.get()), this)
            .until(() -> inputs.stalled))
        .andThen(Commands.runOnce(() -> {
          io.setPercent(0);
          io.resetPosition(0.0);
          calibrated = true;
        }, this))
        .withName("Calibrate Hood");
  }

  public boolean atPosition() {
    if (!calibrated) {
      return false;
    }
    return Math.abs(inputs.position - inputs.setpoint) < hoodPositionTolerance.get();
  }

  public void resetCalibration() {
    calibrated = false;
    io.resetPosition(200);
  }

  private Command calibrateIfNeeded(Command command) {
    return calibrate().onlyIf(() -> !calibrated).andThen(command);
  }

  public boolean calibrated() {
    return calibrated;
  }

  public double getPosition() {
    return inputs.position;
  }
}
