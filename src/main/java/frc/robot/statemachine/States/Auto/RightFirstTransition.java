package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class RightFirstTransition extends State {
    public RightFirstTransition(StateMachineBase stateMachine, Drive drive) {
        super(stateMachine);

        startWhenActive(
                DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getSafeRightNeutral(),
                        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180))));
    }
}
