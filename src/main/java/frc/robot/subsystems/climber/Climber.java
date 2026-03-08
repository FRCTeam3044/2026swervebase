package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import me.nabdev.oxconfig.ConfigurableParameter;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final ConfigurableParameter<Double> speed = new ConfigurableParameter<>(0.0, "Climber speed");
  private final ConfigurableParameter<Double> topPosition = new ConfigurableParameter<>(0.0, "Top climber position");
  private final ConfigurableParameter<Double> bottomPosition = new ConfigurableParameter<>(0.0,
      "Bottom climber position");
  private final ConfigurableParameter<Double> climbPosition = new ConfigurableParameter<>(0.0, "Climb position");
  private final ConfigurableParameter<Double> climberCalibrationSpeed = new ConfigurableParameter<Double>(-0.1,
      "Climber calibration speed");
  private boolean calibrated = false;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command calibrate() {
    return Commands.runEnd(() -> {
      io.setSpeed(climberCalibrationSpeed.get());
      if (io.resetIfPressed())
        calibrated = true;
    }, () -> {
      io.setSpeed(0);
      if (io.resetIfPressed())
        calibrated = true;
    }, this).onlyIf(() -> !calibrated).until(() -> calibrated).withName("Calibrate Climber");
  }

  public Command setSpeed(boolean forward) { // Command factory for setting the speed of climber motor. Use for testing
    return Commands.runEnd(
        () -> {
          io.setSpeed(forward ? speed.get() : -speed.get());
        },
        () -> {
          io.setSpeed(0);
        },
        this);
  }

  public Command setSpeedWParameter(DoubleSupplier speedParameter) {
    return Commands.runEnd(
        () -> {
          io.setSpeed(speedParameter.getAsDouble());
        }, () -> {
          io.setSpeed(0);
        }, this);
  }

  public Command climberTop() { // Command factory for moving climber to top pos
    return Commands.runEnd(
        () -> {
          io.setClimberPos(topPosition.get());
        },
        () -> {
          io.setSpeed(0.0);
        },
        this);
  }

  public Command climberBottom() { // Command factory for moving climber to bottom pos
    return Commands.runEnd(
        () -> {
          io.setClimberPos(bottomPosition.get());
        },
        () -> {
          io.setSpeed(0.0);
        },
        this);
  }

  public Command climberPulledUp() { // Command factory for pulling the climber up to set up pos
    return Commands.runEnd(
        () -> {
          io.setClimberPos(climbPosition.get());
        },
        () -> {
          io.setSpeed(0.0);
        },
        this);
  }
}
