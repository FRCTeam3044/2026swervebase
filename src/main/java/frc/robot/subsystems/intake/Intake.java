package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.HashSet;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private ConfigurableParameter<Double> intakeRollerSpeed = new ConfigurableParameter<Double>(0.5,
      "Intake roller speed");
  private ConfigurableParameter<Double> intakeRollerSpeedSlow = new ConfigurableParameter<Double>(0.05,
      "Intake roller speed slow");

  // private ConfigurableParameter<Double> intakeTopPosition = new
  // ConfigurableParameter<Double>(0.5,
  // "Intake top position");
  private ConfigurableParameter<Double> intakeDeploySpeed = new ConfigurableParameter<Double>(-0.05,
      "Intake deploy speed");
  private ConfigurableParameter<Double> intakeRetractSpeed = new ConfigurableParameter<Double>(0.05,
      "Intake retract speed");

  private ConfigurableParameter<Double> intakeDownTime = new ConfigurableParameter<Double>(0.05,
      "Intake down time auto");
  private ConfigurableParameter<Double> intakeUpTime = new ConfigurableParameter<Double>(0.05,
      "Intake up time auto");

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command intakeTop() {
    return Commands.runEnd(() -> io.setIntakeSpeed(intakeRetractSpeed.get()), () -> io.setIntakeSpeed(0))
        .withName("Intake to Top");
  }

  public Command intakeBottom() {
    return Commands.runEnd(() -> io.setIntakeSpeed(intakeDeploySpeed.get()), () -> io.setIntakeSpeed(0))
        .withName("Intake to Bottom");
  }

  public Command intakeJostle() {
    return Commands.defer(() -> Commands.repeatingSequence(intakeBottom().withTimeout(intakeDownTime.get()),
        intakeTop().withTimeout(intakeUpTime.get())), new HashSet<>());
  }

  public Command runRollers() {
    return Commands.runEnd(() -> io.setSpeedRollers(intakeRollerSpeed.get()), () -> io.setSpeedRollers(0), this)
        .withName("Run Intake Rollers");
  }

  public Command runRollersSlow() {
    return Commands.runEnd(() -> io.setSpeedRollers(intakeRollerSpeedSlow.get()), () -> io.setSpeedRollers(0), this)
        .withName("Run Intake Rollers");
  }

  public Command runRollersJamDetection() {
    return Commands.runEnd(() -> io.setSpeedRollers(intakeRollerSpeed.get()), () -> io.setSpeedRollers(0), this)
        .withName("Run Intake Rollers");
  }

  public Command runRollersReverse() {
    return Commands.runEnd(() -> io.setSpeedRollers(-intakeRollerSpeed.get()), () -> io.setSpeedRollers(0), this)
        .withName("Run Intake Rollers");
  }
}
