package frc.robot.statemachine.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

public class TeleState extends State {

  public static boolean shooterEngaged = true;

  public TeleState(
      StateMachineBase stateMachine,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Climber climber,
      Intake intake,
      Spindexer spindexer,
      Hood hood,
      Turret turret,
      LEDs leds) {
    super(stateMachine);
    SmartXboxController controller = new SmartXboxController(driverController, loop);
    SmartXboxController operator = new SmartXboxController(operatorController, loop);

    startWhenActive(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            true));

    operator.leftTrigger()
        .onTrue(Commands.runOnce(() -> shooterEngaged = !shooterEngaged).withName("Toggle shooter engaged"));

    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    operator.povUp().whileTrue(climber.setSpeed(true));
    operator.povDown().whileTrue(climber.setSpeed(false));
    operator.a().whileTrue(intake.intakeTop());
    operator.a().whileFalse(intake.intakeBottom());
    operator.a().whileFalse(intake.runRollers());

    startWhenActive(intake.intakeBottom().onlyWhile(operatorController.a().negate()));
    startWhenActive(intake.runRollers().onlyWhile(operatorController.a().negate()));

    startWhenActive(spindexer.setSpeed());

    t(() -> turret.inHoodDangerZone() && !hood.calibrated()).whileTrue(turret.exitDangerZone());

    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // ((Arena2026Rebuilt) SimulatedArena.getInstance())
    // .outpostDump(AllianceUtil.getAlliance() == AllianceColor.BLUE))
    // .ignoringDisable(true));

    startWhenActive(leds.setBlinkingOrange());
  }
}
