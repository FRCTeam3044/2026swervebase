package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Command ShooterRPM(DoubleSupplier speed) {
    return Commands.runEnd(() -> io.setSpeed(0.7), () -> io.setSpeed(0));
  }
}
