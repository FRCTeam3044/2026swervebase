package frc.robot.statemachine.States;

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
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood) {
    super(stateMachine);

    startWhenActive(intake.moveIntake(null));
    startWhenActive(intake.runRollers(null));
    startWhenActive(spindexer.setSpeed(0));
  }
}
