package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.function.BooleanSupplier;
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

  public Command setSpeed(DoubleSupplier speed) {
    return setSpeed(speed, speed, () -> false);
  }

  public Command setSpeed() {
    return setSpeed(() -> false);
  }

  public Command setSpeed(boolean reverse) {
    return setSpeed(topRollerSpeed::get, bottomRollerSpeed::get, () -> reverse);
  }

  public Command setSpeed(BooleanSupplier reverse) {
    return setSpeed(topRollerSpeed::get, bottomRollerSpeed::get, reverse);
  }

  public Command setSpeed(DoubleSupplier topSpeed, DoubleSupplier bottomSpeed, BooleanSupplier reverse) {
    return Commands.runEnd(() -> {
      io.setTopSpeed((reverse.getAsBoolean() ? -1.0 : 1.0) * topSpeed.getAsDouble());
      io.setBottomSpeed((reverse.getAsBoolean() ? -1.0 : 1.0) * bottomSpeed.getAsDouble());
    }, () -> {
      io.setTopSpeed(0);
      io.setBottomSpeed(0);
    }, this).withName("Run Spindexer at speed");
  }
}
