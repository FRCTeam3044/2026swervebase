package frc.robot.statemachine.States.ConsolidatedAuto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeDepot extends State {
        public IntakeDepot(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil, AutoAim autoAim, Drive drive,
                        Intake intake,
                        Kicker kicker, Shooter shooter) {
                super(stateMachine);

                Supplier<Pose2d> pathfindingTarget = () -> AutoTargetUtil.getDepot().poseFacing(
                                StateMachine.pathfindingDist.get(),
                                true);
                Supplier<Pose2d> intakeTarget = () -> AutoTargetUtil.getDepot()
                                .poseFacing(StateMachine.intakeDist.get(), true);

                Command pathfind = DriveCommands
                                .goToPointAndthen(drive, pathfindingTarget,
                                                () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)),
                                                () -> DriveCommands.pointControl(drive, pathfindingTarget), true)
                                .withName("Pathfinding");

                Command pointControl = DriveCommands.pointControl(drive, intakeTarget)
                                .until(() -> DriveCommands.pointControllerConverged).withName("Close point control");

                // startWhenActive(autoAim.fire(() -> false)
                // .onlyIf(() -> drive.atPose(pathfindingTarget.get())));
                t(() -> drive.atPose(pathfindingTarget.get())).onTrue(autoAim.fire(() -> false));

                startWhenActive(autoAim.aimHub(() -> true).onlyIf(() -> autoTargetUtil.inAllianceZone()));
                t(() -> autoTargetUtil.inAllianceZone()).onTrue(autoAim.aimHub(() -> true));

                startWhenActive(pathfind);
                t(() -> drive.atPoseTight(pathfindingTarget.get())
                                && drive.atRotationTight(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))))
                                .onTrue(pointControl);
        }
}