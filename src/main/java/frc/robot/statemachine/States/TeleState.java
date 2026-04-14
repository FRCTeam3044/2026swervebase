package frc.robot.statemachine.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
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
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartTrigger;
import me.nabdev.oxidation.util.SmartXboxController;

public class TeleState extends State {

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

    DoubleSupplier driveY = () -> -driverController.getLeftX();
    DoubleSupplier driveX = () -> -driverController.getLeftY();

    startWhenActive(
        DriveCommands.joystickDrive(
            drive,
            driveX,
            driveY,
            () -> -driverController.getRightX(),
            true,
            operator.rightTrigger().or(operator.rightBumper())));
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
        operator.rightTrigger().or(operator.rightBumper())));

    abxy.negate().whileTrue(DriveCommands.joystickDrive(
        drive,
        driveX,
        driveY,
        () -> -driverController.getRightX(),
        true,
        operator.rightTrigger().or(operator.rightBumper())));

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.start().onTrue(Commands.runOnce(hood::resetCalibration));

    // lt = raise but keep intake running
    // a = raise intake and turn off
    // operator.a().whileTrue(intake.intakeTop());
    // operator.a().or(operator.leftTrigger()).whileFalse(intake.intakeBottom());
    // operator.a().or(operator.b()).whileFalse(intake.runRollers());
    // operator.b().whileTrue(intake.runRollersReverse());

    // raise intake when a is pressed
    // operator.a().whileTrue(intake.intakeTop());
    // run intake down unless a or left trigger is pressed
    // operator.a().or(operator.leftTrigger()).whileFalse(intake.intakeBottom());
    // run rollers unless a or b is pressed
    // operator.a().or(operator.b()).whileFalse(intake.runRollers());
    // when b is pressed run rollers in reverse
    operator.b().whileTrue(intake.runRollersReverse());
    // raise intake when trigger is pressed
    operator.leftTrigger().runWhileTrue(intake.intakeTop());
    // when y is pressed, run intake down and spin rollers
    operator.a().and(operator.leftTrigger().negate()).whileTrue(intake.intakeBottom());
    operator.a().whileTrue(intake.runRollers());
    // startWhenActive(
    // intake.intakeBottom()
    // .onlyWhile(operatorController.a().negate().and(operatorController.leftTrigger().negate())));
    // startWhenActive(intake.runRollers().onlyWhile(operatorController.a().negate()));

    // startWhenActive(spindexer.setSpeed());

    t(turret::nearerFlipAround).whileTrue(Commands.runEnd(() -> {
      driverController.setRumble(RumbleType.kBothRumble, 1);
    }, () -> {
      driverController.setRumble(RumbleType.kBothRumble, 0);
    }));

    startWhenActive(leds.defaultPattern());
    t(turret::nearFlipAround).whileTrue(leds.setBlinkingOrSolidColor(() -> {
      return turret.nearestFlipAround() || !turret.nearerFlipAround();
    }, () -> {
      return turret.nearerFlipAround() ? Color.kRed : Color.kOrange;
    }))
        .whileFalse(leds.defaultPattern());

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
