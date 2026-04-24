package frc.robot.statemachine.States.Tele;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.States.TeleState;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.util.SmartXboxController;

public class AlliedZone extends State {
  public AlliedZone(StateMachine stateMachine, CommandXboxController controller,
      CommandXboxController operatorController, Hood hood,
      AutoAim autoAim) {
    super(stateMachine);

    SmartXboxController driver = new SmartXboxController(controller, loop);
    SmartXboxController operator = new SmartXboxController(operatorController, loop);

    operator.povDown().or(() -> !hood.calibrated())
        .whileFalse(autoAim.aimHub(() -> driver.rightTrigger().or(driver.rightBumper()).getAsBoolean()));
    startWhenActive(
        autoAim.aimHub(() -> driver.rightTrigger().or(driver.rightBumper()).getAsBoolean())
            .until(operator.povDown().or(() -> !hood.calibrated())));
  }
}
