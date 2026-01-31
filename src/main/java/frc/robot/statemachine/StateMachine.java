package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.statemachine.States.ActiveHub;
import frc.robot.statemachine.States.AlliedZone;
import frc.robot.statemachine.States.InactiveHub;
import frc.robot.statemachine.States.NeutralZone;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AllianceUtil.AllianceColor;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
  String autoWinner = DriverStation.getGameSpecificMessage();
  AllianceColor allianceColor = AllianceUtil.getAlliance();
  Double currentTime = Robot.timer.get();

  Boolean wonAuto =
      ((autoWinner == "R") && (allianceColor == AllianceColor.RED))
          || ((autoWinner == "B") && (allianceColor == AllianceColor.BLUE));

  Boolean hubIsActive = false; // Replace with actual condition to check if hub is active

  public StateMachine(
      Drive Drive,
      Intake Intake,
      Spindexer Spindexer,
      Kicker Kicker,
      Shooter Shooter,
      Turret Turret,
      Hood Hood) {
    super();

    NeutralZone neutralZone =
        new NeutralZone(this, Drive, Intake, Spindexer, Kicker, Turret, Hood, Shooter);
    AlliedZone alliedZone = new AlliedZone(this);
    ActiveHub activeHub =
        new ActiveHub(this, Drive, Intake, Spindexer, Kicker, Turret, Hood, Shooter);
    InactiveHub inactiveHub = new InactiveHub(this, Drive, Intake, Spindexer, Kicker, Turret, Hood);

    alliedZone.withChild(inactiveHub, () -> false, 0, "Inactive Hub");
    alliedZone.withChild(activeHub, () -> false, 1, "Active Hub");

    alliedZone.withTransition(neutralZone, () -> false, 0, "Drive into Neutral Zone");
    neutralZone.withTransition(alliedZone, () -> false, 0, "Drive into Allied Zone");

    activeHub.withTransition(inactiveHub, () -> false, 0, "Hub becomes inactive");
    inactiveHub.withTransition(activeHub, () -> false, 0, "Hub becomes active");
  }
}
