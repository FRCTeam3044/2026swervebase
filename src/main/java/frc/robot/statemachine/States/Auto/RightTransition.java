package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class RightTransition extends State {
    public RightTransition(StateMachineBase stateMachine, Drive drive) {
        super(stateMachine);

        startWhenActive(
                DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getSafeRightNeutral(),
                        () -> Rotation2d.fromDegrees(180)));
    }
}
