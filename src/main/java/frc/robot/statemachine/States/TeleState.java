package frc.robot.statemachine.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AllianceUtil.AllianceColor;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class TeleState extends State {
  public TeleState(
      StateMachineBase stateMachine,
      CommandXboxController driverController,
      Drive drive,
      LEDs leds) {
    super(stateMachine);
    SmartXboxController controller = new SmartXboxController(driverController, loop);

    startWhenActive(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            true));

    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> ((Arena2026Rebuilt) SimulatedArena.getInstance())
                    .outpostDump(AllianceUtil.getAlliance() == AllianceColor.BLUE))
                .ignoringDisable(true));

    startWhenActive(leds.setBlinkingOrange());
  }
}
