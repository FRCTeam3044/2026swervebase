package frc.robot.statemachine.States.ConsolidatedAuto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class NeutralPaths extends State {
    public static boolean stateComplete = false;

    public NeutralPaths(StateMachineBase stateMachine, AutoAim autoAim, Supplier<ArrayList<Pose2d>> path, Drive drive,
            Kicker kicker, Intake intake, double firstRot, double secondRot, int turnPoint, boolean passing,
            boolean slow) {
        super(stateMachine);

        Supplier<Rotation2d> rot = () -> {
            if (!stateComplete) {
                return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(firstRot));
            } else {
                return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(secondRot));
            }
        };

        Supplier<Pose2d> pose = () -> {
            ArrayList<Pose2d> points = path.get();
            if (points == null || points.size() == 0) {
                return new Pose2d();
            }
            Pose2d lastPose = points.get(points.size() - 1);
            return new Pose2d(lastPose.getTranslation(), rot.get());
        };

        startWhenActive(intake.intakeBottom());
        startWhenActive(intake.runRollers());
        t(() -> drive.atPose(path.get().get(path.get().size() - 1))).onTrue(intake.intakeJostle());

        startWhenActive(Commands.runOnce(() -> stateComplete = false));

        startWhenActive(DriveCommands.goToPointsAndThenSlow(drive, () -> path.get(),
                () -> rot.get(), () -> DriveCommands.pointControl(drive, pose), slow));

        t(() -> drive.atPose(path.get().get(turnPoint)))
                .onTrue(Commands.runOnce(() -> stateComplete = true));

        startWhenActive(autoAim.fire(() -> true).onlyIf(() -> passing));
        startWhenActive(autoAim.aimAllianceZone(() -> true).onlyIf(() -> passing));
    }

}
