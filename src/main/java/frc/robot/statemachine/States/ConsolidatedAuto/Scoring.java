package frc.robot.statemachine.States.ConsolidatedAuto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vertex;

public class Scoring extends State {
        public Scoring(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil, Drive drive, Intake intake,
                        Kicker kicker,
                        Shooter shooter,
                        Hood hood, Turret turret, AutoAim autoAim,
                        double rot) {
                super(stateMachine);

                Supplier<Pose2d> targetSupplier = () -> {
                        if (autoTargetUtil.inAllianceZone()) {
                                return drive.getPose();
                        }
                        Obstacle allianceZone = AutoTargetUtil.allianceSide();
                        Vertex robotPos = new Vertex(drive.getPose());
                        return allianceZone.calculateNearestPoint(robotPos).asPose2d();
                };

                startWhenActive(intake.intakeBottom());
                startWhenActive(intake.runRollers());

                startWhenActive(DriveCommands.goToPoint(drive, targetSupplier,
                                () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(rot)), false));
                t(() -> drive.atPose(targetSupplier.get()))
                                .whileTrue(Commands.run(() -> drive.stop()));
                startWhenActive(() -> Commands
                                .deferredProxy(() -> Commands
                                                .runOnce(() -> RobotContainer.getInstance().autoStateTimer.start()))
                                .onlyIf(() -> autoTargetUtil.inAllianceZone()));
                t(() -> autoTargetUtil.inAllianceZone())
                                .onTrue(Commands.runOnce(() -> RobotContainer.getInstance().autoStateTimer.start()));
                startWhenActive(Commands.waitSeconds(0.4).andThen(autoAim.fire(() -> false))
                                .onlyIf(() -> autoTargetUtil.inAllianceZone() && shooter.isAtSpeed()
                                                && turret.isAtTarget()
                                                && hood.atPosition()));
                t(shooter::isAtSpeed).and(turret::isAtTarget).and(autoTargetUtil::inAllianceZone).and(hood::atPosition)
                                .onTrue(Commands.waitSeconds(0.4).andThen(autoAim.fire(() -> false)));
                startWhenActive(autoAim.aimHub(() -> true).onlyIf(() -> autoTargetUtil.inAllianceZone()));
                t(() -> autoTargetUtil.inAllianceZone()).onTrue(autoAim.aimHub(() -> true));
        }
}
