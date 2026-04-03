package frc.robot.statemachine.States.Tele;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.States.TeleState;
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

public class NeutralZone extends State {
  public NeutralZone(
      StateMachine stateMachine,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      GenericHID operatorBoard,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Hood hood,
      Shooter shooter,
      AutoAim autoAim, LEDs leds) {
    super(stateMachine);
    SmartXboxController driver = new SmartXboxController(driverController, loop);
    SmartXboxController operator = new SmartXboxController(operatorController, loop);

    driver.start().whileFalse(autoAim.aimAllianceZone(() -> !operatorController.x().getAsBoolean()));
    startWhenActive(
        autoAim.aimAllianceZone(() -> !operatorController.x().getAsBoolean()).until(driver.start()));
    driver.rightTrigger().whileTrue(autoAim.fire(() -> true));
    startWhenActive(leds.defaultPattern());
  }
}
