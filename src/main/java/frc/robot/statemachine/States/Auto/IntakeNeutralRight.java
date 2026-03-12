package frc.robot.statemachine.States.Auto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeNeutralRight extends State {
    public static boolean stateComplete = false;

    public IntakeNeutralRight(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil,
            Supplier<ArrayList<Pose2d>> waypoints, AutoAim autoAim, Drive drive,
            Intake intake) {
        super(stateMachine);

        Supplier<Rotation2d> rot = () -> {
            if (autoTargetUtil.inAllianceZone()) {
                return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180));
            } else if (autoTargetUtil.inNeutralZone()) {
                if (!stateComplete) {
                    return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(300));
                } else {
                    return AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(50));
                }
            } else {
                return Rotation2d.fromDegrees(0);
            }
        };

        startWhenActive(Commands.runOnce(() -> stateComplete = false));
        startWhenActive(
                DriveCommands.goToPoints(drive, waypoints, rot));
        t(() -> autoTargetUtil.inNeutralZone()).onTrue(intake.intakeBottom());
        t(() -> autoTargetUtil.inNeutralZone()).onTrue(intake.runRollers());
        t(() -> drive.atPose(waypoints.get().get(1)))
                .onTrue(Commands.runOnce(() -> stateComplete = true));

        startWhenActive(autoAim.aimAllianceZone(() -> false).onlyIf(() -> !stateComplete).withName("Shoot to AZ"));
        t(() -> !stateComplete).whileTrue(autoAim.aimAllianceZone(() -> false).withName("Shoot to AZ"));
    }
}