package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.statemachine.States.CalibrationState;
import frc.robot.statemachine.States.DisabledState;
import frc.robot.statemachine.States.NormalTestState;
import frc.robot.statemachine.States.Tele.ActiveHub;
import frc.robot.statemachine.States.Tele.AlliedZone;
import frc.robot.statemachine.States.Tele.InactiveHub;
import frc.robot.statemachine.States.Tele.NeutralZone;
import frc.robot.statemachine.States.TeleState;
import frc.robot.statemachine.States.TestState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoAimDataManager;
import frc.robot.util.AllianceUtil.AllianceColor;
import frc.robot.util.AutoTargetUtil;
import java.util.function.BooleanSupplier;

import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
        String autoWinner = DriverStation.getGameSpecificMessage();
        AllianceColor allianceColor = AllianceUtil.getAlliance();

        public StateMachine(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        GenericHID operatorBoard,
                        Drive drive,
                        Intake intake,
                        Spindexer spindexer,
                        Kicker kicker,
                        Shooter shooter,
                        Turret turret,
                        Hood hood,
                        Climber climber,
                        LEDs leds,
                        AutoTargetUtil autoTargetUtil,
                        AutoAim autoAim) {
                super();

                Timer timer = Robot.timer;

                BooleanSupplier transitionShift = () -> timer.get() >= 0 && timer.get() < 10;
                BooleanSupplier shiftOne = () -> timer.get() >= 10 && timer.get() < 35;
                BooleanSupplier shiftTwo = () -> timer.get() >= 35 && timer.get() < 60;
                BooleanSupplier shiftThree = () -> timer.get() >= 60 && timer.get() < 85;
                BooleanSupplier shiftFour = () -> timer.get() >= 85 && timer.get() < 110;
                BooleanSupplier endGame = () -> timer.get() >= 110;

                BooleanSupplier wonAuto = () -> {
                        String autoWinner = DriverStation.getGameSpecificMessage();
                        AllianceColor allianceColor = AllianceUtil.getAlliance();
                        return ((autoWinner == "R") && (allianceColor == AllianceColor.RED))
                                        || ((autoWinner == "B") && (allianceColor == AllianceColor.BLUE));
                };

                BooleanSupplier hubIsActive = () -> {
                        return AllianceUtil.getAlliance() == AllianceColor.UNKNOWN
                                        || transitionShift.getAsBoolean()
                                        || endGame.getAsBoolean()
                                        || (wonAuto.getAsBoolean()
                                                        && (shiftTwo.getAsBoolean() || shiftFour.getAsBoolean()))
                                        || (!wonAuto.getAsBoolean()
                                                        && (shiftOne.getAsBoolean() || shiftThree.getAsBoolean()));
                };

                DisabledState disabled = new DisabledState(this, leds);
                currentState = disabled;
                State teleop = new TeleState(this, driverController, drive, leds);
                State test = new TestState(this);

                this.registerToRootState(test, teleop, disabled);
                // Test States
                State calibration = new CalibrationState(this, driverController, drive, shooter, turret, hood, kicker,
                                spindexer, autoTargetUtil);
                State normalTest = new NormalTestState(this, driverController, operatorController, drive, hood, intake,
                                kicker,
                                shooter, spindexer, turret, climber, leds);
                test.withDefaultChild(calibration).withChild(normalTest,
                                () -> !AutoAimDataManager.calibrationMode.get(), 0,
                                "Normal Test Mode");
                calibration.withTransition(normalTest, () -> !AutoAimDataManager.calibrationMode.get(), 0,
                                "Exit Calibration Mode");
                normalTest.withTransition(calibration, AutoAimDataManager.calibrationMode::get, 0,
                                "Enter Calibration Mode");

                // Teleop States
                AlliedZone alliedZone = new AlliedZone(this);
                NeutralZone neutralZone = new NeutralZone(
                                this, driverController, operatorBoard, drive, intake, spindexer, kicker, turret, hood,
                                shooter,
                                autoAim);
                ActiveHub activeHub = new ActiveHub(
                                this, driverController, operatorBoard, drive, intake, spindexer, kicker, turret, hood,
                                shooter,
                                autoAim);
                InactiveHub inactiveHub = new InactiveHub(this, driverController, drive, intake, spindexer, kicker,
                                turret,
                                hood);

                teleop.withDefaultChild(alliedZone).withChild(neutralZone);

                alliedZone.withChild(activeHub, hubIsActive, 0, "Active Hub");
                alliedZone.withChild(inactiveHub, () -> !hubIsActive.getAsBoolean(), 1, "Inactive Hub");

                alliedZone.withTransition(
                                neutralZone, () -> autoTargetUtil.inNeutralZone(), 0, "Drive into Neutral Zone");
                neutralZone.withTransition(
                                alliedZone, () -> autoTargetUtil.inAllianceZone(), 0, "Drive into Allied Zone");

                activeHub.withTransition(
                                inactiveHub, () -> !hubIsActive.getAsBoolean(), 0, "Hub becomes inactive");
                inactiveHub.withTransition(activeHub, hubIsActive, 0, "Hub becomes active");

                teleop.withModeTransitions(disabled, teleop, test);
                test.withModeTransitions(disabled, teleop, test);
                disabled.withModeTransitions(disabled, teleop, test);
        }
}
