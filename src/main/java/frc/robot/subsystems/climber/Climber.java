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

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
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
    return Commands.run(
        () -> {
          io.setSpeed(speedParameter.getAsDouble());
        });
  }

  public Command climberTop() { // Command factory for moving climber to top pos
    return Commands.run(
        () -> {
          io.setClimberPos(topPosition.get());
        },
        this);
  }

  public Command climberBottom() { // Command factory for moving climber to bottom pos
    return Commands.run(
        () -> {
          io.setClimberPos(bottomPosition.get());
        },
        this);
  }

  public Command climberPulledUp() { // Command factory for pulling the climber up to set up pos
    return Commands.run(
        () -> {
          io.setClimberPos(climbPosition.get());
        },
        this);
  }
}
