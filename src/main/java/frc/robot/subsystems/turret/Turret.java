package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command setAngle(Supplier<Angle> angle) {
    return Commands.runEnd(() -> io.setAngle(angle.get()), () -> io.setPercent(0), this)
        .withName("Set Turret Angle");
  }

  public Command runPercent(DoubleSupplier percent) {
    return Commands.runEnd(() -> io.setPercent(percent.getAsDouble()), () -> io.setPercent(0), this)
        .withName("Run Turret At Percent");
  }

  public Angle getAngle() {
    return inputs.angle;
  }
}
