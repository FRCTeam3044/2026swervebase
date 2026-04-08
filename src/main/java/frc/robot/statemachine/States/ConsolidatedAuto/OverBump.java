package frc.robot.statemachine.States.ConsolidatedAuto;

import frc.robot.subsystems.drive.Drive;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class OverBump extends State {
    public OverBump(StateMachineBase stateMachine, Drive drive) {
        super(stateMachine);
    }
}
