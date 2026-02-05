package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private ConfigurableParameter<Double> intakeRollerSpeed = new ConfigurableParameter<Double>(0.5,
      "Intake roller speed");

  private ConfigurableParameter<Double> intakeTopPosition = new ConfigurableParameter<Double>(0.5,
      "Intake top position");
  private ConfigurableParameter<Double> intakeBottomPosition = new ConfigurableParameter<Double>(0.5,
      "Intake bottom position");

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command intakeTop() {
    return Commands.run(() -> io.setIntakePosition(intakeTopPosition.get()), this);
  }

  public Command intakeBottom() {
    return Commands.run(() -> io.setIntakePosition(intakeBottomPosition.get()), this);
  }

  public Command runRollers() {
    return Commands.run(() -> io.setSpeedRollers(intakeRollerSpeed.get()), this);
  }
}
