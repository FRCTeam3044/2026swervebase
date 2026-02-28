package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class AutoClimb extends State {
  public AutoClimb(StateMachineBase stateMachine, Drive drive) {
    super(stateMachine);

    startWhenActive(
        DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getLeftTower(), () -> Rotation2d.fromDegrees(0)));
    t(() -> drive.getPose().getTranslation()
        .getDistance(AutoTargetUtil.getLeftTower().getTranslation()) < DriveCommands.pathfindingTolerance.get())
        .whileTrue(Commands.run(drive::stop));
  }
}
