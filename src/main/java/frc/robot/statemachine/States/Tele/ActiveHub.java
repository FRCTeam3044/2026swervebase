package frc.robot.statemachine.States.Tele;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.util.SmartXboxController;

public class ActiveHub extends State {
  public ActiveHub(
      StateMachine stateMachine,
      CommandXboxController driverController,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood,
      Shooter shooter) {
    super(stateMachine);
    SmartXboxController controller = new SmartXboxController(driverController, loop);

    startWhenActive(intake.intakeBottom());
    controller.leftTrigger().whileTrue(intake.runRollers());
    startWhenActive(spindexer.run());
    // startWhenActive(kicker.runKicker());
    // startWhenActive(hood.moveHood());
    // startWhenActive(turret.rotate());
    // controller.rightTrigger().whileTrue(shooter.runShooter());
  }
}
