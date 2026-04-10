package frc.robot.statemachine.States.ConsolidatedAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeOutpost extends State {
        public IntakeOutpost(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil, AutoAim autoAim, Drive drive,
                        Intake intake,
                        Kicker kicker, Shooter shooter) {
                super(stateMachine);

                startWhenActive(
                                DriveCommands.goToPointSlow(drive, () -> AutoTargetUtil.getOutpost(),
                                                () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))));
                t(() -> drive.atPose(AutoTargetUtil.getOutpost())).onTrue(Commands.run(() -> drive.stop()));

                startWhenActive(autoAim.fire(() -> false)
                                .onlyIf(() -> autoTargetUtil.inAllianceZone() && shooter.isAtSpeed()));
                t(shooter::isAtSpeed).and(autoTargetUtil::inAllianceZone).onTrue(autoAim.fire(() -> false));
                startWhenActive(autoAim.aimHub(() -> true).onlyIf(() -> autoTargetUtil.inAllianceZone()));
                t(() -> autoTargetUtil.inAllianceZone()).onTrue(autoAim.aimHub(() -> true));
        }
}
