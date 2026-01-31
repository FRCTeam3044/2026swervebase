package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private final ConfigurableParameter<Double> speedOne =
      new ConfigurableParameter<>(0.0, "Kicker motor 1 speed");

  private final ConfigurableParameter<Double> speedTwo =
      new ConfigurableParameter<>(0.0, "Kicker motor 2 speed");

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Command runKicker() {
    return Commands.run(() -> io.setSpeed(speedOne.get(), speedTwo.get()), this);
  }
}
