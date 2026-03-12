package frc.robot.statemachine.States;

import frc.robot.subsystems.LEDs.LEDs;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class DisabledState extends State {
  public DisabledState(StateMachineBase stateMachine, LEDs leds) {
    super(stateMachine);

    startWhenActive(leds.defaultPattern());
  }
}
