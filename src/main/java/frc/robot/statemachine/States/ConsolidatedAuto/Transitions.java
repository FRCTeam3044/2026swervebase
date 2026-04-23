package frc.robot.statemachine.States.ConsolidatedAuto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class Transitions extends State {
        public Transitions(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil, AutoAim autoAim, Drive drive,
                        Intake intake,
                        Hood hood, Turret turret, Shooter shooter, Supplier<Pose2d> pos,
                        double rot, boolean shooting, boolean forceFast) {
                super(stateMachine);

                Supplier<Command> pointControllerCommand = () -> DriveCommands.pointControl(drive, () -> {
                        Pose2d pose = pos.get();
                        return new Pose2d(pose.getX(), pose.getY(),
                                        AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(rot)));
                });

                startWhenActive(
                                DriveCommands.goToPointAndthen(drive, () -> pos.get(),
                                                () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(rot)),
                                                pointControllerCommand, forceFast));
                t(() -> !autoTargetUtil.inAllianceZone()).onTrue(
                                DriveCommands.goToPointAndthen(drive, () -> pos.get(),
                                                () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(rot)),
                                                pointControllerCommand,
                                                forceFast));

                // startWhenActive(() -> pointControllerCommand.get().onlyIf(() ->
                // pointControl));
                // t(() -> drive.atPose(pos.get())).onTrue(DriveCommands.pointControl(drive,
                // pos));

                startWhenActive(intake.intakeBottom());
                startWhenActive(intake.runRollers());

                startWhenActive(autoAim.fire(() -> !autoTargetUtil.inAllianceZone()).onlyIf(() -> shooting));

                startWhenActive(
                                autoAim.aimHub(() -> true).onlyWhile(() -> autoTargetUtil.inAllianceZone() && shooting)
                                                .withName("Aim Hub (trans)"));
                startWhenActive(
                                autoAim.aimAllianceZone(() -> true)
                                                .onlyWhile(() -> !autoTargetUtil.inAllianceZone() && shooting)
                                                .withName("Aim Az (trans)"));
                t(() -> autoTargetUtil.inAllianceZone() && shooting).runWhileTrue(autoAim.aimHub(() -> true))
                                .runWhileFalse(autoAim.aimAllianceZone(() -> true));
        }
}
