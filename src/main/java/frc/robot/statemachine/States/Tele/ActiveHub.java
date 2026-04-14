package frc.robot.statemachine.States.Tele;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.LEDs.LEDs;
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
      Shooter shooter, AutoAim autoAim, LEDs leds) {
    super(stateMachine);
    SmartXboxController operator = new SmartXboxController(operatorController, loop);
    SmartXboxController driver = new SmartXboxController(driverController, loop);
    startWhenActive(autoAim.fire(() -> operator.rightBumper().getAsBoolean(), () -> operator.x()
        .getAsBoolean())
        .onlyWhile(operator.rightTrigger().or(operator.rightBumper())));
    operator.rightTrigger().or(operator.rightBumper())
        .whileTrue(autoAim.fire(() -> operator.rightBumper().getAsBoolean(), () -> operator.x().getAsBoolean()));
    // .whileFalse(leds.setBlinkingColor(Color.kGreen));
    // driver.rightTrigger().negate().and(turret::isAtTarget).and(shooter::isAtSpeed)
    // .whileTrue(leds.setSolidColor(() -> Color.kGreen));
    // t(() -> turret.isAtTarget()).or(() ->
    // !shooter.isAtSpeed()).whileTrue(leds.setBlinkingColor(Color.kYellow));

  }
}
