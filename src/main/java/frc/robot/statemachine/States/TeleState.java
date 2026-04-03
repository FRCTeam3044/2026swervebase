package frc.robot.statemachine.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.HubShiftUtil;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartTrigger;
import me.nabdev.oxidation.util.SmartXboxController;

public class TeleState extends State {
  private ConfigurableParameter<Double> slowModeSpeed = new ConfigurableParameter<>(0.6, "Slow Mode Speed");

  public TeleState(
      StateMachineBase stateMachine,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
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
            true,
            operator.rightTrigger()));
    SmartTrigger abxy = controller.a().or(controller.b()).or(controller.x()).or(controller.y());

    DoubleSupplier targetRotation = () -> {
      if (controller.a().getAsBoolean()) {
        return 0;
      } else if (controller.b().getAsBoolean()) {
        return 90.0;
      } else if (controller.x().getAsBoolean()) {
        return 270.0;
      } else if (controller.y().getAsBoolean()) {
        return 180.0;
      } else {
        return 0.0;
      }
    };
    abxy.whileTrue(DriveCommands.joystickDriveAtAngle(drive, driveX, driveY,
        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(targetRotation.getAsDouble())),
        operator.rightTrigger()));

    abxy.negate().whileTrue(DriveCommands.joystickDrive(
        drive,
        driveX,
        driveY,
        () -> -driverController.getRightX() * slowMult.getAsDouble(),
        true,
        operator.rightTrigger()));

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.start().onTrue(Commands.runOnce(hood::resetCalibration));

    operator.a().whileTrue(intake.intakeTop());
    operator.a().or(operator.x()).whileFalse(intake.intakeBottom());
    operator.a().or(operator.b()).whileFalse(intake.runRollers());
    operator.b().whileTrue(intake.runRollersReverse());
    operator.x().whileTrue(intake.intakeTop());

    startWhenActive(
        intake.intakeBottom().onlyWhile(operatorController.a().negate().and(operatorController.x().negate())));
    startWhenActive(intake.runRollers().onlyWhile(operatorController.a().negate()));

    // startWhenActive(spindexer.setSpeed());

    t(turret::nearFlipAround).whileTrue(Commands.runEnd(() -> {
      driverController.setRumble(RumbleType.kBothRumble, 1);
    }, () -> {
      driverController.setRumble(RumbleType.kBothRumble, 0);
    }));

    t(() -> HubShiftUtil.getShiftedShiftInfo().remainingTime() < 10)
        .onTrue(Commands.deadline(Commands.waitSeconds(0.25), Commands.runEnd(() -> {
          operatorController.setRumble(RumbleType.kBothRumble, 1);
        }, () -> {
          operatorController.setRumble(RumbleType.kBothRumble, 0);
        })));

    t(() -> HubShiftUtil.getShiftedShiftInfo().remainingTime() < 5)
        .onTrue(Commands.deadline(Commands.waitSeconds(0.75), Commands.runEnd(() -> {
          operatorController.setRumble(RumbleType.kBothRumble, 1);
        }, () -> {
          operatorController.setRumble(RumbleType.kBothRumble, 0);
        })));

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
