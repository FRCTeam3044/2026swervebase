package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command setPosition(DoubleSupplier position) {
    return Commands.runEnd(
        () -> io.setPosition(position.getAsDouble()),
        () -> io.setPercent(0),
        this)
        .withName("Set Hood Position");
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(
        () -> io.setPercent(percent.getAsDouble()),
        () -> io.setPercent(0),
        this)
        .withName("Run Hood At Percent");
  }

  public double getPosition() {
    return inputs.position;
  }
}
