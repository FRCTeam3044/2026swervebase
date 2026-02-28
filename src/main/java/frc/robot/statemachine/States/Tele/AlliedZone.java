package frc.robot.statemachine.States.Tele;

import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.States.TeleState;
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;

public class AlliedZone extends State {
  public AlliedZone(StateMachine stateMachine, AutoAim autoAim) {
    super(stateMachine);

    startWhenActive(
        autoAim.aimHub(() -> TeleState.shooterEngaged));
  }
}
