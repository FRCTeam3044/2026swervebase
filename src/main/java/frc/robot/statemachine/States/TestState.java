package frc.robot.statemachine.States;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import java.util.function.DoubleSupplier;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

public class TestState extends State {
  public TestState(
      StateMachineBase stateMachine,
      CommandXboxController TestController,
      Drive drive,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter,
      Spindexer spindexer,
      Turret turret) {
    super(stateMachine);
    SmartXboxController testController = new SmartXboxController(TestController, loop);
    DoubleSupplier rightX = () -> -MathUtil.applyDeadband(TestController.getRightX(), 0.1);
    DoubleSupplier leftY = () -> -MathUtil.applyDeadband(TestController.getLeftY(), 0.1);
    DoubleSupplier leftX = () -> -MathUtil.applyDeadband(TestController.getLeftX(), 0.1);

    testController.a().whileTrue(hood.moveHood());
    testController.b().whileTrue(intake.intakeTop());
    testController.x().whileTrue(intake.intakeBottom());
    testController.y().whileTrue(intake.runRollers());
    testController.leftBumper().whileTrue(kicker.runKicker());
    testController.rightBumper().whileTrue(shooter.runShooter());
    testController.leftTrigger().whileTrue(spindexer.setSpeed());
    // testController.rightTrigger().whileTrue(turret.rotate(null));
    Command joystickDrive = DriveCommands.joystickDrive(drive, leftY, leftX, rightX, false);
    startWhenActive(joystickDrive);

    /*
     * Button Binding:
     * A = Hood Move
     * B = Intake Top Move
     * X = Intake Bottom Move
     * Y = Intake Roller Spin
     * LB = Kicker Move
     * RB = Shooter shoot
     * LT = Spindexer spin
     * RT = Turret rotate
     * Joystick = Drive
     */
  }
}
