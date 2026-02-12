package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private final ConfigurableParameter<Double> topShootSpeed =
      new ConfigurableParameter<>(0.0, "Kicker top shoot speed");
  private final ConfigurableParameter<Double> topBlockSpeed =
      new ConfigurableParameter<>(0.0, "Kicker top block speed");

  private final ConfigurableParameter<Double> bottomSpeed =
      new ConfigurableParameter<>(0.0, "Kicker bottom speed");

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Command shootKicker() {
    return Commands.run(
            () -> {
              io.setTopPercent(topShootSpeed.get());
              io.setBottomPercent(bottomSpeed.get());
            },
            this)
        .withName("Run Kicker");
  }

  public Command blockKicker() {
    return Commands.run(
            () -> {
              io.setTopPercent(topBlockSpeed.get());
              io.setBottomPercent(bottomSpeed.get());
            },
            this)
        .withName("Block Kicker");
  }
}
