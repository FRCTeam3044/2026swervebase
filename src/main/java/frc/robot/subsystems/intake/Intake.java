package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command moveIntake(DoubleSupplier speedIntake) {
    return Commands.run(() -> io.setSpeedIntake(speedIntake.getAsDouble()), this);
  }

  public Command runRollers(DoubleSupplier speedRollers) {
    return Commands.run(() -> io.setSpeedRollers(speedRollers.getAsDouble()), this);
  }
}
