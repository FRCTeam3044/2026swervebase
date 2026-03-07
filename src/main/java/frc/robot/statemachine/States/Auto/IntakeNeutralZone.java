package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

    Command leftToRight = Commands.sequence(DriveCommands.goToPoint(drive,
        () -> AutoTargetUtil.getLeftNeutral(), () -> Rotation2d.fromDegrees(45)),
        DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getRightNeutral(), () -> Rotation2d.fromDegrees(0)));

    Command rightToLeft = Commands.sequence(DriveCommands.goToPoint(drive,
        () -> AutoTargetUtil.getRightNeutral(), () -> Rotation2d.fromDegrees(45)),
        DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getLeftNeutral(), () -> Rotation2d.fromDegrees(0)));

    startWhenActive(rightToLeft);
  }
}
