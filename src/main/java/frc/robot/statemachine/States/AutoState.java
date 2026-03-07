package frc.robot.statemachine.States;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class AutoState extends State {
  public AutoState(StateMachineBase stateMachine, Turret turret, Hood hood) {
    super(stateMachine);

    t(() -> turret.inHoodDangerZone() && !hood.calibrated()).whileTrue(turret.exitDangerZone());
  }
}
