package frc.robot.statemachine.States;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class SysIDState extends State {
    public SysIDState(StateMachineBase stateMachine, LoggedDashboardChooser<Command> chooser) {
        super(stateMachine);
        startWhenActive(chooser::get);
    }

    @Override
    public void onEnter() {
        super.onEnter();
    }
}