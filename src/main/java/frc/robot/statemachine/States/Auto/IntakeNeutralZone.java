package frc.robot.statemachine.States.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeNeutralZone extends State {
  public IntakeNeutralZone(StateMachineBase stateMachine, Drive drive, Intake intake) {
    super(stateMachine);

    startWhenActive(
        DriveCommands.goToPoint(drive, () -> AutoTargetUtil.getLeftNeutral(), () -> Rotation2d.fromDegrees(0)));
    startWhenActive(intake.intakeBottom());
    startWhenActive(intake.runRollers());
  }
}
