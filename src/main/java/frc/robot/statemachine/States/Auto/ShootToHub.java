package frc.robot.statemachine.States.Auto;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vertex;

public class ShootToHub extends State {
  public ShootToHub(
      StateMachineBase stateMachine,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood,
      Shooter shooter,
      AutoAim autoAim) {
    super(stateMachine);

    Supplier<Pose2d> targetSupplier = () -> {
      Obstacle allianceZone = AutoTargetUtil.allianceSide();
      Vertex robotPos = new Vertex(drive.getPose());
      return allianceZone.calculateNearestPoint(robotPos).asPose2d();
    };

    BooleanSupplier atPosition = () -> {
      double distance = drive.getPose().getTranslation().getDistance(targetSupplier.get().getTranslation());
      return distance < DriveCommands.pathfindingTolerance.get();
    };

    startWhenActive(DriveCommands.goToPoint(drive, targetSupplier, () -> Rotation2d.fromDegrees(0)));
    startWhenActive(intake.intakeBottom());
    startWhenActive(spindexer.run());
    startWhenActive(kicker.blockKicker());
    t(atPosition).whileTrue(Commands.run(() -> drive.stop()));
    t(atPosition)
        .onTrue(Commands.deferredProxy(() -> Commands.runOnce(RobotContainer.getInstance().autoStateTimer::restart)));
    t(atPosition).whileTrue(kicker.shootKicker());
    t(atPosition).whileTrue(autoAim.aimHub(() -> true));
  }
}
