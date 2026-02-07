package frc.robot.statemachine.States.Tele;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxidation.State;

public class InactiveHub extends State {
  public InactiveHub(
      StateMachine stateMachine,
      CommandXboxController driverController,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood) {
    super(stateMachine);

    startWhenActive(
        Commands.waitSeconds(1)
            .deadlineFor(
                Commands.runEnd(
                    () -> driverController.setRumble(RumbleType.kBothRumble, 1),
                    () -> driverController.setRumble(RumbleType.kBothRumble, 0))));
    startWhenActive(intake.intakeBottom());
    t(driverController.leftTrigger()).whileTrue(intake.runRollers());
    startWhenActive(spindexer.setSpeed());
    startWhenActive(kicker.runKicker());
  }
}
