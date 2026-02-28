package frc.robot.statemachine.States.Auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeNeutralZone extends State {
  public IntakeNeutralZone(StateMachineBase stateMachine, Drive drive, Intake intake) {
    super(stateMachine);

    BooleanSupplier atFirstPosition = () -> {
      double distance = drive.getPose().getTranslation().getDistance(AutoTargetUtil.getLeftNeutral().getTranslation());
      return distance < DriveCommands.pathfindingTolerance.get();
    };

    BooleanSupplier atSecondPosition = () -> {
      double distance = drive.getPose().getTranslation().getDistance(AutoTargetUtil.getRightNeutral().getTranslation());
      return distance < DriveCommands.pathfindingTolerance.get();
    };

    startWhenActive(
        DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getLeftNeutral(), () -> Rotation2d.fromDegrees(0)));
    t(atFirstPosition)
        .onTrue(
            DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getRightNeutral(), () -> Rotation2d.fromDegrees(0)));
    t(atSecondPosition).whileTrue(Commands.run(() -> drive.stop()));
    startWhenActive(intake.intakeBottom());
    startWhenActive(intake.runRollers());
  }
}
