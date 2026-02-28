package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.statemachine.States.Auto.AutoClimb;
import frc.robot.statemachine.States.Auto.IntakeAllianceZone;
import frc.robot.statemachine.States.Auto.IntakeNeutralZone;
import frc.robot.statemachine.States.Auto.ShootToAlliedSide;
import frc.robot.statemachine.States.Auto.ShootToHub;
import frc.robot.statemachine.States.AutoState;
import frc.robot.statemachine.States.CalibrationState;
import frc.robot.statemachine.States.DisabledState;
import frc.robot.statemachine.States.NormalTestState;
// import frc.robot.statemachine.States.SysIDState;
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
import frc.robot.util.AutoEnums.AutoSteps;
import frc.robot.util.AutoTargetUtil;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
        String autoWinner = DriverStation.getGameSpecificMessage();
        AllianceColor allianceColor = AllianceUtil.getAlliance();

        ArrayList<AutoSteps> testAutoRoutine = new ArrayList<AutoSteps>();

        AutoSteps currentStep;
        int index;

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
                        AutoAim autoAim,
                        LoggedDashboardChooser<Command> chooser) {
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
                State auto = new AutoState(this);

                this.registerToRootState(test, teleop, disabled, auto);
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

                // Autonomous work

                AutoClimb autoClimb = new AutoClimb(this, drive);
                IntakeAllianceZone intakeAllianceZone = new IntakeAllianceZone(this, drive);
                IntakeNeutralZone intakeNeutralZone = new IntakeNeutralZone(this, drive, intake);
                ShootToAlliedSide shootToAlliedSide = new ShootToAlliedSide(this, drive);
                ShootToHub shootHub = new ShootToHub(this, drive, intake, spindexer, kicker, turret, hood, shooter,
                                autoAim);

                auto.withDefaultChild(shootHub)
                                .withChild(autoClimb)
                                .withChild(intakeAllianceZone)
                                .withChild(intakeNeutralZone)
                                .withChild(shootToAlliedSide);

                testAutoRoutine.add(AutoSteps.ShootToHub);
                testAutoRoutine.add(AutoSteps.IntakeNeutralZone);
                testAutoRoutine.add(AutoSteps.ShootToHub);
                testAutoRoutine.add(AutoSteps.AutoClimb);
                currentStep = testAutoRoutine.get(index);

                BooleanSupplier currentStateComplete = () -> {
                        if (currentStep.getCondition().getAsBoolean()) {
                                index = index + 1;
                                currentStep = testAutoRoutine.get(index);
                                return true;
                        } else {
                                return false;
                        }
                };

                autoClimb.withTransition(auto, currentStateComplete, 0, "Climb to auto");
                shootHub.withTransition(auto, currentStateComplete, 0, "Shoot hub to auto");
                shootToAlliedSide.withTransition(auto, currentStateComplete, 0, "Neutral shot to auto");
                intakeAllianceZone.withTransition(auto, currentStateComplete, 0, "Allied intake to shot");
                intakeNeutralZone.withTransition(auto, currentStateComplete, 0, "Neutral intake to shot");

                auto.withTransition(autoClimb, () -> currentStep == AutoSteps.AutoClimb, 0, "Auto to climb");
                auto.withTransition(shootHub, () -> currentStep == AutoSteps.ShootToHub, 0, "Auto to hub shot");
                auto.withTransition(shootToAlliedSide, () -> currentStep == AutoSteps.ShootToAlliedSide, 0,
                                "Auto to neutral shot");
                auto.withTransition(intakeAllianceZone, () -> currentStep == AutoSteps.IntakeAllianceZone, 0,
                                "Auto to allied intake");
                auto.withTransition(intakeNeutralZone,
                                () -> currentStep == AutoSteps.IntakeNeutralZone, 0, "Auto to neutral intake");

                // For SYSID (comment out for normal autos)
                // MAKE SURE YOU ADD AUTO TO THE REGISTER TO ROOT STATE

                // SysIDState auto = new SysIDState(this, chooser);
                // teleop.withModeTransitions(disabled, teleop, auto, test);
                // test.withModeTransitions(disabled, teleop, auto, test);
                // disabled.withModeTransitions(disabled, teleop, auto, test);
                // auto.withModeTransitions(disabled, teleop, auto, test);
        }
}
