package frc.robot.statemachine.States.ConsolidatedAuto;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class OverBump extends State {
    public final ConfigurableParameter<Double> bumpSpeed = new ConfigurableParameter<>(-0.7, "Over bump speed");

    public OverBump(StateMachineBase stateMachine, Drive drive) {
        super(stateMachine);

        startWhenActive(DriveCommands.joystickDrive(drive, () -> bumpSpeed.get(), () -> 0, () -> 0, true, () -> false));
    }
}
