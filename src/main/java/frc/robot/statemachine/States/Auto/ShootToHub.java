package frc.robot.statemachine.States.Auto;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class ShootToHub extends State {
  public ShootToHub(
      StateMachineBase stateMachine,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood,
      Shooter shooter) {
    super(stateMachine);

        // Drive into alliance zone
        startWhenActive(intake.intakeTop());
        startWhenActive(spindexer.run());
        startWhenActive(kicker.runKicker());
        startWhenActive(hood.moveHood());
        startWhenActive(turret.setAngle(null));
        t(/* Robot is in alliance zone */ () -> false).onTrue(shooter.runShooter());

        
    }
}
