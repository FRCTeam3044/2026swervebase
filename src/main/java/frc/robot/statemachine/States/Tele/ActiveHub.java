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
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.util.SmartXboxController;

public class ActiveHub extends State {

  public ActiveHub(
      StateMachine stateMachine,
      CommandXboxController driverController,
      // GenericHID operatorBoard,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood,
      Shooter shooter, AutoAim autoAim) {
    super(stateMachine);
    SmartXboxController operator = new SmartXboxController(operatorController, loop);
    operator.rightTrigger().whileTrue(kicker.shootKicker());
  }
}