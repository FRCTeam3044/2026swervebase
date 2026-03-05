package frc.robot.statemachine.States.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import frc.robot.util.AutoTargetUtil.POIData;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class AutoClimb extends State {
        private static ConfigurableParameter<Double> pathfindingDist = new ConfigurableParameter<>(0.7,
                        "Climb pathfinding distance");
        public static ConfigurableParameter<Double> preScoreDist = new ConfigurableParameter<>(0.5,
                        "Pre Climb distance");
        private static ConfigurableParameter<Double> scoreDist = new ConfigurableParameter<>(0.0,
                        "Climb distance");

        public AutoClimb(StateMachineBase stateMachine, AutoTargetUtil autoTargetUtil, POIData climbSide,
                        AutoAim autoAim, Drive drive,
                        Climber climber) {
                super(stateMachine);

                Supplier<Pose2d> pathfindingTarget = () -> climbSide.poseFacing(pathfindingDist.get(), false);
                Supplier<Pose2d> farTarget = () -> climbSide.poseFacing(preScoreDist.get(), false);
                Supplier<Pose2d> scoreTarget = () -> climbSide.poseFacing(scoreDist.get(), false);

                Command pathfind = DriveCommands.goToPoint(drive, pathfindingTarget, () -> Rotation2d.fromDegrees(0))
                                .withName("Pathfinding");

                Command far = DriveCommands.pointControl(drive, farTarget)
                                .until(() -> DriveCommands.pointControllerConverged).withName("Far point control");

                Command close = DriveCommands.pointControl(drive, scoreTarget)
                                .until(() -> DriveCommands.pointControllerConverged).withName("Close point control");

                startWhenActive(pathfind);
                startWhenActive(climber.climberTop().onlyIf(() -> autoTargetUtil.inAllianceZone())
                                .withName("Climber to top"));
                t(() -> drive.atPose(pathfindingTarget.get())).onTrue(far);
                t(() -> drive.atPose(farTarget.get())).onTrue(close);
                t(() -> drive.atPose(scoreTarget.get())).onTrue(climber.climberPulledUp());
                t(() -> autoTargetUtil.inAllianceZone()).onTrue(autoAim.aimHub(() -> true));
        }
}
