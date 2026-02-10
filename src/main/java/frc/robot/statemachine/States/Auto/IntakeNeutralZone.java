package frc.robot.statemachine.States.Auto;

import frc.robot.subsystems.intake.Intake;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeNeutralZone extends State {
  public IntakeNeutralZone(StateMachineBase stateMachine, Intake intake) {
    super(stateMachine);

    // Drive to neutral zone

    startWhenActive(intake.intakeBottom());
  }
}
