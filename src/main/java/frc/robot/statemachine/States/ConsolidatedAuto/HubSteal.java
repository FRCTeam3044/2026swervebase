package frc.robot.statemachine.States.ConsolidatedAuto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class HubSteal extends State {
    public HubSteal(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil, AutoAim autoAim, Drive drive,
            boolean left) {
        super(stateMachine);

        Supplier<Pose2d> sideTarget = left ? AutoTargetUtil::getLeftMidline : AutoTargetUtil::getRightMidline;

        startWhenActive(DriveCommands
                .goToPointAndthen(drive, () -> AutoTargetUtil.getOppositeHubPoint(),
                        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180)),
                        () -> DriveCommands.pointControl(drive, () -> AutoTargetUtil.getOppositeHubPoint()), true)
                .withName("Hub steal path"));

        t(() -> drive.atPose(AutoTargetUtil.getOppositeHubPoint())).onTrue(DriveCommands.goToPointAndthen(drive,
                sideTarget, () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)),
                () -> DriveCommands.pointControl(drive, sideTarget),
                true));

        startWhenActive(autoAim.fire(() -> true));
        startWhenActive(autoAim.aimAllianceZone(() -> true));
    }
}
