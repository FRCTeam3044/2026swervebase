package frc.robot.statemachine.States;

import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class TestState extends State {
    public TestState(StateMachineBase stateMachine, Hood hood, Turret turret, LEDs leds) {
        super(stateMachine);

        // t(() -> turret.inHoodDangerZone() &&
        // !hood.calibrated()).whileTrue(turret.exitDangerZone());
        startWhenActive(leds.defaultPattern());
    }
}
