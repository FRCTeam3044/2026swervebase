package frc.robot.statemachine.States;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import java.util.List;
import java.util.function.DoubleSupplier;
import me.nabdev.oxconfig.ConfigurableClass;
import me.nabdev.oxconfig.ConfigurableClassParam;
import me.nabdev.oxconfig.OxConfig;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

public class NormalTestState extends State implements ConfigurableClass {
        private final ConfigurableClassParam<Double> shooterSpeed = new ConfigurableClassParam<>(this, 0.0,
                        "Test Shooter Speed");
        private final ConfigurableClassParam<Double> turretPosition = new ConfigurableClassParam<>(this, 0.0,
                        "Test Turret Position");
        private final ConfigurableClassParam<Double> hoodPosition = new ConfigurableClassParam<>(this, 0.0,
                        "Test Hood Position");

        public NormalTestState(
                        StateMachineBase stateMachine,
                        CommandXboxController controllerOne,
                        CommandXboxController controllerTwo,
                        Drive drive,
                        Hood hood,
                        Intake intake,
                        Kicker kicker,
                        Shooter shooter,
                        Spindexer spindexer,
                        Turret turret,
                        Climber climber,
                        LEDs leds) {

                super(stateMachine);
                SmartXboxController testControllerOne = new SmartXboxController(controllerOne, loop);
                SmartXboxController testControllerTwo = new SmartXboxController(controllerTwo, loop);

                /*
                 * Button Bindings:
                 *
                 * When Controller One: Joysticks Drive
                 * When Controller Two: Joysticks Move Subsystems Manually
                 *
                 * Aiming Subsystems:
                 * - Controller One: Press button to go to test position
                 * - Controller Two: Press button and move joystick to set speed of subsystem
                 * A = Hood (left joystick Y)
                 * B = Shooter (Right joystick Y)
                 * X = Turret (Left joystick X)
                 * Y = Spindexer (Right joystick X)
                 * POV Right = Climber Manual Move (Right joystick Y)
                 *
                 * Other Subsystems:
                 * LB = Intake Top
                 * RB = Intake Bottom
                 * LT = Kicker Shoot
                 * RT = Kicker Block
                 *
                 * POV Up = Climber Top
                 * POV Down = Climber Down
                 * POV Right = Climber ClimbPosition
                 * POV Left = Intake Rollers
                 *
                 */

                //

                DoubleSupplier rightY = () -> -MathUtil.applyDeadband(controllerTwo.getRightY(),
                                DriveCommands.DEADBAND);
                DoubleSupplier leftY = () -> -MathUtil.applyDeadband(controllerTwo.getLeftY(), DriveCommands.DEADBAND);
                DoubleSupplier leftX = () -> -MathUtil.applyDeadband(controllerTwo.getLeftX(), DriveCommands.DEADBAND);
                DoubleSupplier rightX = () -> -MathUtil.applyDeadband(controllerTwo.getRightX(),
                                DriveCommands.DEADBAND);

                testControllerOne.a().whileTrue(hood.setPosition(hoodPosition::get));
                testControllerOne.b().whileTrue(shooter.runSpeed(() -> RPM.of(shooterSpeed.get())));
                testControllerOne.x().whileTrue(turret.setAngle(() -> Degrees.of(turretPosition.get())));
                testControllerOne.y().whileTrue(spindexer.setSpeed());

                testControllerOne.povUp().whileTrue(climber.climberTop());
                testControllerOne
                                .povRight()
                                .whileTrue(climber.climberPulledUp());
                testControllerOne
                                .povDown()
                                .whileTrue(climber.climberBottom());
                testControllerTwo
                                .povRight()
                                .whileTrue(climber.setSpeedWParameter(rightY));

                testControllerTwo.a().whileTrue(hood.runPercent(leftY));
                testControllerTwo.b().whileTrue(shooter.runPercent(rightY));
                testControllerTwo.x().whileTrue(turret.runPercent(leftX));
                testControllerTwo.y().whileTrue(spindexer.setSpeed(rightX));

                testControllerOne.leftBumper().or(testControllerTwo.leftBumper()).whileTrue(intake.intakeTop());
                testControllerOne
                                .rightBumper()
                                .or(testControllerTwo.rightBumper())
                                .whileTrue(intake.intakeBottom());
                testControllerOne.povLeft().or(testControllerTwo.povLeft()).whileTrue(intake.runRollers());
                testControllerOne
                                .leftTrigger()
                                .or(testControllerTwo.leftTrigger())
                                .whileTrue(kicker.shootKicker());
                testControllerOne
                                .rightTrigger()
                                .or(testControllerTwo.rightTrigger())
                                .whileTrue(kicker.blockKicker());

                // testController.rightTrigger().whileTrue(turret.rotate(null));
                startWhenActive(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -controllerOne.getLeftY(),
                                                () -> -controllerOne.getLeftX(),
                                                () -> -controllerOne.getRightX(),
                                                false));
                startWhenActive(leds.defaultPattern());
                startWhenActive(hood.calibrate());
                // startWhenActive(turret.exitDangerZone().onlyIf(hood::calibrationNeeded)
                // .until(() -> !hood.calibrationNeeded()));
                OxConfig.registerConfigurableClass(this);
        }

        @Override
        public List<ConfigurableClassParam<?>> getParameters() {
                return List.of(shooterSpeed, turretPosition, hoodPosition);
        }

        @Override
        public String getKey() {
                return "TestMode";
        }
}
