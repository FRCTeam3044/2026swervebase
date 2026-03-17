package frc.robot.statemachine.States.ConsolidatedAuto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class NeutralPaths extends State {
    public static boolean stateComplete = false;

    public NeutralPaths(StateMachineBase stateMachine, AutoAim autoAim, ArrayList<Pose2d> path, Drive drive,
            Kicker kicker, double firstRot, double secondRot, int turnPoint) {
        super(stateMachine);

        Supplier<Rotation2d> rot = () -> {
            if (!stateComplete) {
                return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(firstRot));
            } else {
                return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(secondRot));
            }
        };

        startWhenActive(DriveCommands.goToPoints(drive, () -> path,
                () -> AllianceUtil.getRotForAlliance(rot.get())));

        t(() -> drive.atPose(path.get(turnPoint)))
                .onTrue(Commands.runOnce(() -> stateComplete = true));

        startWhenActive(kicker.shootKicker());
        startWhenActive(autoAim.aimAllianceZone(() -> true));
    }

}
