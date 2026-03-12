package frc.robot.statemachine.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.util.AllianceUtil;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartTrigger;
import me.nabdev.oxidation.util.SmartXboxController;

public class TeleState extends State {
  private ConfigurableParameter<Double> slowModeSpeed = new ConfigurableParameter<>(0.6, "Slow Mode Speed");

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

    DoubleSupplier slowMult = () -> (operator.rightTrigger().getAsBoolean()
        && !driverController.rightTrigger().getAsBoolean()) ? slowModeSpeed.get() : 1;
    DoubleSupplier driveY = () -> -driverController.getLeftX() * slowMult.getAsDouble();
    DoubleSupplier driveX = () -> -driverController.getLeftY() * slowMult.getAsDouble();
    startWhenActive(
        DriveCommands.joystickDrive(
            drive,
            driveX,
            driveY,
            () -> -driverController.getRightX() * slowMult.getAsDouble(),
            true));
    SmartTrigger abxy = controller.a().or(controller.b()).or(controller.x()).or(controller.y());

    DoubleSupplier targetRotation = () -> {
      // a = 180, y = 0, b = 90, x = 270
      if (controller.a().getAsBoolean()) {
        return 180.0;
      } else if (controller.b().getAsBoolean()) {
        return 90.0;
      } else if (controller.x().getAsBoolean()) {
        return 270.0;
      } else if (controller.y().getAsBoolean()) {
        return 0.0;
      } else {
        return 180.0;
      }
    };
    abxy.whileTrue(DriveCommands.joystickDriveAtAngle(drive, driveX, driveY,
        () -> AllianceUtil.getRotForAlliance(new Rotation2d(targetRotation.getAsDouble()))));

    abxy.negate().whileTrue(DriveCommands.joystickDrive(
        drive,
        driveX,
        driveY,
        () -> -driverController.getRightX() * slowMult.getAsDouble(),
        true));

    operator.leftTrigger()
        .onTrue(Commands.runOnce(() -> shooterEngaged = !shooterEngaged).withName("Toggle shooter engaged"));

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.start().onTrue(Commands.runOnce(hood::resetCalibration));

    operator.povUp().whileTrue(climber.setSpeed(true));
    operator.povDown().whileTrue(climber.setSpeed(false));
    operator.a().whileTrue(intake.intakeTop());
    operator.a().whileFalse(intake.intakeBottom());
    operator.a().whileFalse(intake.runRollers());

    startWhenActive(intake.intakeBottom().onlyWhile(operatorController.a().negate()));
    startWhenActive(intake.runRollers().onlyWhile(operatorController.a().negate()));

    startWhenActive(spindexer.setSpeed());

    // t(() -> turret.inHoodDangerZone() &&
    // !hood.calibrated()).whileTrue(turret.exitDangerZone());

    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // ((Arena2026Rebuilt) SimulatedArena.getInstance())
    // .outpostDump(AllianceUtil.getAlliance() == AllianceColor.BLUE))
    // .ignoringDisable(true));

    startWhenActive(leds.defaultPattern());
  }
}
