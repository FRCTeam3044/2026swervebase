package frc.robot.statemachine.States.ConsolidatedAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoAim;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class OverBump extends State {
    public final ConfigurableParameter<Double> bumpSpeed = new ConfigurableParameter<>(-0.7, "Over bump speed");

    public OverBump(StateMachineBase stateMachine, AutoAim autoAim, Drive drive) {
        super(stateMachine);

        startWhenActive(Commands.runOnce(() -> {
            drive.bumpTimer.stop();
            drive.bumpTimer.reset();
            drive.pastBump = false;
        }));
        startWhenActive(DriveCommands.joystickDrive(drive, () -> bumpSpeed.get(), () -> 0, () -> 0, true, () -> false)
                .withName("Drive over bump"));

        startWhenActive(autoAim.aimHub(() -> true));
    }
}
