package frc.robot.statemachine.States.Auto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeNeutralZone extends State {
    public static boolean stateComplete = false;

    public IntakeNeutralZone(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil,
            Supplier<ArrayList<Pose2d>> waypoints, Drive drive,
            Intake intake) {
        super(stateMachine);

        Supplier<Rotation2d> rot = () -> {
            if (autoTargetUtil.inAllianceZone()) {
                return Rotation2d.fromDegrees(0);
            } else if (autoTargetUtil.inNeutralZone()) {
                return Rotation2d.fromDegrees(45);
            } else {
                return Rotation2d.fromDegrees(0);
            }
        };

        startWhenActive(
                DriveCommands.goToPoints(drive, waypoints, rot));
        t(() -> autoTargetUtil.inNeutralZone()).onTrue(intake.intakeBottom());
        t(() -> autoTargetUtil.inNeutralZone()).onTrue(intake.runRollers());
        t(() -> drive.atPose(waypoints.get().get(2)))
                .onTrue(Commands.runOnce(() -> stateComplete = true));
    }
}