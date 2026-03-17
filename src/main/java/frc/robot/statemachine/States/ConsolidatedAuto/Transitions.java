package frc.robot.statemachine.States.ConsolidatedAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class Transitions extends State {
    public Transitions(StateMachineBase stateMachine, Drive drive, Pose2d pos,
            double rot) {
        super(stateMachine);

        startWhenActive(
                DriveCommands.goToPoint(drive, () -> pos,
                        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(rot))));
    }
}
