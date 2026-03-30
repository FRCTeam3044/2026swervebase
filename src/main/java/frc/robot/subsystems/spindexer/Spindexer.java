package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private ConfigurableParameter<Double> topRollerSpeed = new ConfigurableParameter<Double>(-0.5, "Top roller speed");
  private ConfigurableParameter<Double> bottomRollerSpeed = new ConfigurableParameter<Double>(0.5,
      "Bottom roller speed");

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public Command setSpeed() {
    return Commands.runEnd(() -> {
      io.setTopSpeed(topRollerSpeed.get());
      io.setBottomSpeed(bottomRollerSpeed.get());
    }, () -> {
      io.setTopSpeed(0);
      io.setBottomSpeed(0);
    }, this)
        .withName("Run Spindexer");
  }

  public Command setSpeed(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      io.setTopSpeed(speed.getAsDouble());
      io.setBottomSpeed(speed.getAsDouble());
    }, () -> {
      io.setTopSpeed(0);
      io.setBottomSpeed(0);
    }, this).withName("Run Spindexer at speed");
  }
}
