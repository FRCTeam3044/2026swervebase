package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  Boolean transitionShift = currentTime >= 0 && currentTime < 10;
  Boolean shiftOne = currentTime >= 10 && currentTime < 35;
  Boolean shiftTwo = currentTime >= 35 && currentTime < 60;
  Boolean shiftThree = currentTime >= 60 && currentTime < 85;
  Boolean shiftFour = currentTime >= 85 && currentTime < 110;
  Boolean endGame = currentTime >= 110;

  Boolean wonAuto =
      ((autoWinner == "R") && (allianceColor == AllianceColor.RED))
          || ((autoWinner == "B") && (allianceColor == AllianceColor.BLUE));

  Boolean hubIsActive =
      (wonAuto && (shiftTwo || shiftFour)) || (!wonAuto && (shiftOne || shiftThree)) || endGame;

  public StateMachine(
      CommandXboxController driverController,
      Drive Drive,
      Intake Intake,
      Spindexer Spindexer,
      Kicker Kicker,
      Shooter Shooter,
      Turret Turret,
      Hood Hood) {
    super();

    AlliedZone alliedZone = new AlliedZone(this);
    NeutralZone neutralZone =
        new NeutralZone(
            this, driverController, Drive, Intake, Spindexer, Kicker, Turret, Hood, Shooter);
    ActiveHub activeHub =
        new ActiveHub(
            this, driverController, Drive, Intake, Spindexer, Kicker, Turret, Hood, Shooter);
    InactiveHub inactiveHub =
        new InactiveHub(this, driverController, Drive, Intake, Spindexer, Kicker, Turret, Hood);

    alliedZone.withChild(activeHub, () -> hubIsActive, 0, "Active Hub");
    alliedZone.withChild(inactiveHub, () -> !hubIsActive, 1, "Inactive Hub");

    alliedZone.withTransition(neutralZone, () -> false, 0, "Drive into Neutral Zone");
    neutralZone.withTransition(alliedZone, () -> false, 0, "Drive into Allied Zone");

    activeHub.withTransition(inactiveHub, () -> !hubIsActive, 0, "Hub becomes inactive");
    inactiveHub.withTransition(activeHub, () -> hubIsActive, 0, "Hub becomes active");
  }
}
