package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.statemachine.States.ConsolidatedAuto.AutoTrajectories;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class FarLeftInOut extends State {
    public FarLeftInOut(StateMachineBase stateMachine, AutoAim autoAim, Drive drive,
            Kicker kicker) {
        super(stateMachine);

        startWhenActive(
                DriveCommands.goToPoints(drive, () -> AutoTrajectories.getFarLeftInOut(),
                        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(90))));

        startWhenActive(kicker.shootKicker());
        startWhenActive(autoAim.aimAllianceZone(() -> true));
    }
}
