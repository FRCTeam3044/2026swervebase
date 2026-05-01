package frc.robot.statemachine.States;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.HapticsUtil;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoTargetUtil;
import frc.robot.util.HubShiftUtil;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartTrigger;
import me.nabdev.oxidation.util.SmartXboxController;

public class TeleState extends State {

  boolean turretEnabled = true;
  boolean overdriveEnabled = false;

  public TeleState(
      StateMachineBase stateMachine,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Hood hood,
      Turret turret,
      Shooter shooter,
      AutoAim autoAim,
      LEDs leds,
      AutoTargetUtil autoTargetUtil) {
    super(stateMachine);
    SmartXboxController driver = new SmartXboxController(driverController, loop);
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
            driver.rightTrigger()/* .or(driver.rightBumper()) */));
    SmartTrigger abxy = driver.a().or(driver.b()).or(driver.x()).or(driver.y());

    DoubleSupplier targetRotation = () -> {
      if (driver.a().getAsBoolean()) {
        return 0;
      } else if (driver.b().getAsBoolean()) {
        return 90.0;
      } else if (driver.x().getAsBoolean()) {
        return 270.0;
      } else if (driver.y().getAsBoolean()) {
        return 180.0;
      } else {
        return 0.0;
      }
    };
    abxy.whileTrue(DriveCommands.joystickDriveAtAngle(drive, driveX, driveY,
        () -> AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(targetRotation.getAsDouble())),
        driver.rightTrigger()/* .or(driver.rightBumper()) */));

    abxy.negate().whileTrue(DriveCommands.joystickDrive(
        drive,
        driveX,
        driveY,
        () -> -driverController.getRightX(),
        true,
        driver.rightTrigger()/* .or(driver.rightBumper()) */));

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    operator.povDown().onTrue(Commands.runOnce(hood::resetCalibration));
    operator.povUp().whileTrue(turret.nudge());
    operator.povLeft().onTrue(Commands.runOnce(() -> {
      turretEnabled = !turretEnabled;
      turret.setEnabled(turretEnabled);
      hood.setEnabled(turretEnabled);
    }));

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
    operator.leftBumper().whileTrue(intake.intakeJostle());
    // raise intake when trigger is pressed
    operator.leftTrigger().runWhileTrue(intake.intakeTop());
    operator.leftTrigger().whileTrue(Commands.runEnd(() -> driverController.setRumble(RumbleType.kBothRumble, 1),
        () -> driverController.setRumble(RumbleType.kBothRumble, 0)));
    // when y is pressed, run intake down and spin rollers
    operator.a().and(operator.leftTrigger().negate()).and((operator.leftBumper().negate()))
        .whileTrue(intake.intakeBottom());
    operator.a().whileTrue(intake.runRollers());
    t(drive::isDrivingTowardsIntake).and(operator.b().negate()).whileTrue(intake.runRollers());
    t(drive::isDrivingTowardsIntake).and(operator.leftBumper().negate()).and(operator.leftTrigger().negate())
        .whileTrue(intake.intakeBottom());
    t(() -> operator.x().getAsBoolean() || driver.rightBumper().getAsBoolean()).and(driver.rightTrigger().negate())
        /* .and(driver.rightBumper().negate()) */
        .whileTrue(spindexer.setSpeed(() -> true));
    // startWhenActive(
    // intake.intakeBottom()
    // .onlyWhile(operatorController.a().negate().and(operatorController.leftTrigger().negate())));
    // startWhenActive(intake.runRollers().onlyWhile(operatorController.a().negate()));

    startWhenActive(leds.defaultPattern());
    BooleanSupplier safeShootBlocking = () -> !autoAim
        .safeShoot(() -> /* driver.rightBumper().getAsBoolean() || */ autoTargetUtil.inNeutralZone())
        && driver.rightTrigger()/* .or(driver.rightBumper()) */.getAsBoolean();
    t(() -> turret.nearFlipAround() || safeShootBlocking.getAsBoolean())
        .whileTrue(leds.setSolidColor(() -> safeShootBlocking.getAsBoolean() ? Color.kGreen
            : (turret.nearerFlipAround() ? Color.kRed : Color.kOrange)));
    t(() -> !turret.nearFlipAround()).and(() -> !safeShootBlocking.getAsBoolean())
        .whileTrue(leds.defaultPattern());

    // operator.rightTrigger().or(operator.rightBumper())
    // .onTrue(HapticsUtil.rumbleForTime(driverController, 0.75))
    // .onFalse(HapticsUtil.rumblePulses(driverController, 2, 0.2, 0.1));

    // t(() -> HubShiftUtil.getShiftedShiftInfo().remainingTime() < 10)
    // .onTrue(HapticsUtil.rumblePulses(operatorController, 3, 0.12, 0.08));
    // t(() -> HubShiftUtil.getShiftedShiftInfo().remainingTime() < 5)
    // .onTrue(Commands.deadline(Commands.waitSeconds(0.75), Commands.runEnd(() -> {
    // operatorController.setRumble(RumbleType.kBothRumble, 1);
    // }, () -> {
    // operatorController.setRumble(RumbleType.kBothRumble, 0);
    // })));

    driver.povLeft().or(driver.povDown()).onTrue(Commands.runOnce(() -> {
      overdriveEnabled = !overdriveEnabled;
      drive.setOverdriveEnabled(overdriveEnabled);
    }));

    startWhenActive(Commands.run(() -> SmartDashboard.putBoolean("OverdriveEnabled", overdriveEnabled)));

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
