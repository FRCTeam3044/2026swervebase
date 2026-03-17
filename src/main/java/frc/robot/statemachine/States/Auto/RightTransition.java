package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class RightTransition extends State {
    public RightTransition(StateMachineBase stateMachine, Drive drive, AutoAim autoAim, Kicker kicker) {
        super(stateMachine);

        startWhenActive(
                DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getSafeRightNeutral(),
                        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))));

        startWhenActive(kicker.shootKicker().withName("Running kicker"));
        startWhenActive(autoAim.aimAllianceZone(() -> true).withName("Shoot to AZ"));
    }
}
