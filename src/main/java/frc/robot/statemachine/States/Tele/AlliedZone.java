package frc.robot.statemachine.States.Tele;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.States.TeleState;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AutoAim;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.util.SmartXboxController;

public class AlliedZone extends State {
  public AlliedZone(StateMachine stateMachine, CommandXboxController controller, CommandXboxController operator,
      AutoAim autoAim) {
    super(stateMachine);

    SmartXboxController driver = new SmartXboxController(controller, loop);

    driver.start().whileFalse(autoAim.aimHub(() -> operator.rightTrigger().or(operator.rightBumper()).getAsBoolean()));
    startWhenActive(
        autoAim.aimHub(() -> operator.rightTrigger().or(operator.rightBumper()).getAsBoolean()).until(driver.start()));
  }
}
