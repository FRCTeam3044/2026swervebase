package frc.robot.statemachine.States.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeOutpost extends State {
    private static ConfigurableParameter<Double> pathfindingDist = new ConfigurableParameter<>(0.7,
            "Outpost pathfinding distance");
    public static ConfigurableParameter<Double> preIntakeDist = new ConfigurableParameter<>(0.5,
            "Pre Outpost Intake distance");
    private static ConfigurableParameter<Double> intakeDist = new ConfigurableParameter<>(0.4,
            "Outpost distance");

    public IntakeOutpost(StateMachineBase stateMachine, Drive drive, Intake intake) {
        super(stateMachine);

        Supplier<Pose2d> pathfindingTarget = () -> AutoTargetUtil.getOutpost().poseFacing(pathfindingDist.get(), false);
        Supplier<Pose2d> farTarget = () -> AutoTargetUtil.getOutpost().poseFacing(preIntakeDist.get(), false);
        Supplier<Pose2d> intakeTarget = () -> AutoTargetUtil.getOutpost().poseFacing(intakeDist.get(), false);

        Command pathfind = DriveCommands.goToPoint(drive, pathfindingTarget, () -> Rotation2d.fromDegrees(0))
                .withName("Pathfinding");

        Command far = DriveCommands.pointControl(drive, farTarget)
                .until(() -> DriveCommands.pointControllerConverged).withName("Far point control");

        Command close = DriveCommands.pointControl(drive, intakeTarget)
                .until(() -> DriveCommands.pointControllerConverged).withName("Close point control");

        startWhenActive(intake.runRollers());
        startWhenActive(pathfind);
        t(() -> drive.atPose(pathfindingTarget.get())).onTrue(far);
        t(() -> drive.atPose(farTarget.get())).onTrue(close);
    }
}
