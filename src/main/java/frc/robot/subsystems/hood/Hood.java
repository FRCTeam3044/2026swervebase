package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final ConfigurableParameter<Double> calibrationSpeed = new ConfigurableParameter<Double>(-0.05,
      "Hood Calibration Speed");

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
    return Commands.runEnd(
        () -> io.setPosition(position.getAsDouble()), () -> io.setPercent(0), this).withName("Set Hood Position");
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(() -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0), this)
        .withName("Run Hood At Percent");
  }

  public Command calibrate() {
    return Commands.runOnce(() -> io.resetPosition(200.0))
        .andThen(Commands.waitUntil(() -> !RobotContainer.getInstance().turret.inHoodDangerZone()))
        .andThen(Commands.run(() -> io.setPercent(calibrationSpeed.get()), this)
            .until(() -> inputs.stalled))
        .andThen(Commands.runOnce(() -> {
          io.setPercent(0);
          io.resetPosition(0.0);
        }, this))
        .withName("Calibrate Hood")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public boolean calibrationNeeded() {
    return calibrated;
  }

  public double getPosition() {
    return inputs.position;
  }
}
