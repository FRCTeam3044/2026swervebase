package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final ConfigurableParameter<Double> angle =
      new ConfigurableParameter<>(0.0, "Hood angle");

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command moveHood() {
    return Commands.run(
            () -> {
              io.setAngle(angle.get());
            },
            this)
        .withName("Set Hood Angle");
  }

  public double getHoodAngle() {
    return inputs.hoodAngleRads;
  }
}
