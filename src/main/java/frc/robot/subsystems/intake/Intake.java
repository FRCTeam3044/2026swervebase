package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final ConfigurableParameter<Double> speedIntake =
      new ConfigurableParameter<>(0.0, "Intake speed");

  private final ConfigurableParameter<Double> speedRollers =
      new ConfigurableParameter<>(0.0, "Roller speed");

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command moveIntake() {
    return Commands.run(() -> io.setSpeedIntake(speedIntake.get()), this);
  }

  public Command runRollers() {
    return Commands.run(() -> io.setSpeedRollers(speedRollers.get()), this);
  }
}
