package frc.robot.statemachine.States.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vertex;

public class IntakeAllianceZone extends State {
    public IntakeAllianceZone(StateMachineBase stateMachine, Drive drive) {
        super(stateMachine);

        Supplier<Pose2d> targetSupplier = () -> {
            Obstacle allianceZone = AutoTargetUtil.allianceSide();
            Vertex robotPos = new Vertex(drive.getPose());
            return allianceZone.calculateNearestPoint(robotPos).asPose2d();
        };

        startWhenActive(DriveCommands.goToPoint(drive, targetSupplier, () -> Rotation2d.fromDegrees(0)));
    }
}