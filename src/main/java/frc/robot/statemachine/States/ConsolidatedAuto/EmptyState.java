package frc.robot.statemachine.States.ConsolidatedAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class EmptyState extends State {
    public EmptyState(StateMachineBase stateMachine, Drive drive) {
        super(stateMachine);

        startWhenActive(Commands.run(() -> drive.stop()));
    }
}