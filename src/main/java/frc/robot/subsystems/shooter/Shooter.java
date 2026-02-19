package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command runSpeed(Supplier<AngularVelocity> speed) {
    return Commands.runEnd(() -> io.setSpeed(speed.get()), () -> io.setPercent(0.0), this)
        .withName("Run Shooter At Speed");
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(
        () -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0.0), this)
        .withName("Run Shooter At Percent");
  }

  public AngularVelocity getSpeed() {
    return inputs.leaderVelocity;
  }
}
