package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
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
import frc.robot.statemachine.States.Auto.AutoClimb;
import frc.robot.statemachine.States.ConsolidatedAuto.AutoTrajectories;
import frc.robot.statemachine.States.ConsolidatedAuto.EmptyState;
import frc.robot.statemachine.States.ConsolidatedAuto.IntakeDepot;
import frc.robot.statemachine.States.ConsolidatedAuto.IntakeOutpost;
import frc.robot.statemachine.States.ConsolidatedAuto.NeutralPaths;
import frc.robot.statemachine.States.ConsolidatedAuto.Scoring;
import frc.robot.statemachine.States.ConsolidatedAuto.Transitions;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoAim;
import frc.robot.util.AutoAimDataManager;
import frc.robot.util.AutoEnums.AutoSteps;
import frc.robot.util.AllianceUtil.AllianceColor;
import frc.robot.util.AutoTargetUtil;
import frc.robot.util.HubShiftUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
        String autoWinner = DriverStation.getGameSpecificMessage();
        AllianceColor allianceColor = AllianceUtil.getAlliance();

        // Autos to choose from
        public static ArrayList<AutoSteps> orbitAuto = new ArrayList<AutoSteps>();

        public static ArrayList<AutoSteps> leftSwoopDepot = new ArrayList<AutoSteps>();
        public static ArrayList<AutoSteps> rightSwoopOutpost = new ArrayList<AutoSteps>();

        public static ArrayList<AutoSteps> leftDoubleSweep = new ArrayList<AutoSteps>();
        public static ArrayList<AutoSteps> rightDoubleSweep = new ArrayList<AutoSteps>();

        public static ArrayList<AutoSteps> justShoot = new ArrayList<AutoSteps>();
        public static ArrayList<AutoSteps> shootDepot = new ArrayList<AutoSteps>();
        public static ArrayList<AutoSteps> shootOutpost = new ArrayList<AutoSteps>();

        public static ArrayList<AutoSteps> leftInOutAuto = new ArrayList<AutoSteps>();
        public static ArrayList<AutoSteps> rightInOutAuto = new ArrayList<AutoSteps>();

        private AutoSteps currentStep;
        private static int index;

        private ConfigurableParameter<Boolean> forceEnableShooting = new ConfigurableParameter<>(false,
                        "Force Enable Shooting");

        Supplier<ArrayList<AutoSteps>> autoSupplier;

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
                        Vision vision,
                        LoggedDashboardChooser<Command> chooser,
                        LoggedDashboardChooser<ArrayList<AutoSteps>> autoChooser) {
                super();

                DisabledState disabled = new DisabledState(this, leds, turret, hood, vision);
                currentState = disabled;
                State teleop = new TeleState(this, driverController, operatorController, drive, climber, intake,
                                spindexer, hood, turret, leds);
                State test = new TestState(this, hood, turret);
                State auto = new AutoState(this, autoChooser, turret, hood, intake, spindexer);

                this.registerToRootState(test, teleop, auto, disabled);

                // Test States
                State calibration = new CalibrationState(this, driverController, drive, shooter, turret, hood, kicker,
                                spindexer, intake, autoTargetUtil);
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
                AlliedZone alliedZone = new AlliedZone(this, driverController, operatorController, autoAim);
                NeutralZone neutralZone = new NeutralZone(
                                this, driverController, operatorController, operatorBoard, drive, intake, spindexer,
                                kicker, turret, hood,
                                shooter,
                                autoAim, leds);
                ActiveHub activeHub = new ActiveHub(
                                this, driverController, operatorController, drive, intake, spindexer, kicker, turret,
                                hood,
                                shooter,
                                autoAim, leds);
                InactiveHub inactiveHub = new InactiveHub(this, driverController, drive, intake, spindexer, kicker,
                                turret,
                                hood, leds);

                teleop.withDefaultChild(alliedZone).withChild(neutralZone);

                alliedZone.withChild(activeHub,
                                () -> HubShiftUtil.getShiftedShiftInfo().active() || forceEnableShooting.get(), 0,
                                "Active Hub");
                alliedZone.withChild(inactiveHub,
                                () -> !HubShiftUtil.getShiftedShiftInfo().active() && !forceEnableShooting.get(), 1,
                                "Inactive Hub");

                alliedZone.withTransition(
                                neutralZone, () -> autoTargetUtil.inNeutralZone(), 0, "Drive into Neutral Zone");
                neutralZone.withTransition(
                                alliedZone, () -> autoTargetUtil.inAllianceZone(), 0, "Drive into Allied Zone");

                activeHub.withTransition(
                                inactiveHub,
                                () -> !HubShiftUtil.getShiftedShiftInfo().active() && !forceEnableShooting.get(), 0,
                                "Hub becomes inactive");
                inactiveHub.withTransition(activeHub,
                                () -> HubShiftUtil.getShiftedShiftInfo().active() || forceEnableShooting.get(), 0,
                                "Hub becomes active");

                teleop.withModeTransitions(disabled, teleop, auto, test);
                test.withModeTransitions(disabled, teleop, auto, test);
                disabled.withModeTransitions(disabled, teleop, auto, test);
                auto.withModeTransitions(disabled, teleop, auto, test);

                // Autonomous work
                AutoClimb leftClimb = new AutoClimb(this, autoTargetUtil, AutoTargetUtil.getLeftTower(), autoAim, drive,
                                climber);
                AutoClimb rightClimb = new AutoClimb(this, autoTargetUtil, AutoTargetUtil.getRightTower(), autoAim,
                                drive,
                                climber);

                // Consolidated auto work
                EmptyState emptyState = new EmptyState(this, drive);

                Scoring firstScore = new Scoring(this, autoTargetUtil, drive, kicker, shooter, hood, turret, autoAim,
                                180);
                Scoring secondScore = new Scoring(this, autoTargetUtil, drive, kicker, shooter, hood, turret, autoAim,
                                0);

                IntakeOutpost intakeOutpost = new IntakeOutpost(this, autoTargetUtil, autoAim, drive, intake, kicker,
                                shooter);
                IntakeDepot intakeDepot = new IntakeDepot(this, autoTargetUtil, autoAim, drive, intake, kicker,
                                shooter);

                Transitions leftTransition = new Transitions(this, drive,
                                AutoTargetUtil.getSafeLeftNeutral(), 0.0);
                Transitions rightTransition = new Transitions(this, drive,
                                AutoTargetUtil.getSafeRightNeutral(), 0.0);

                NeutralPaths leftSwoop = new NeutralPaths(this, autoAim, () -> AutoTrajectories.getLeftCurve(), drive,
                                kicker,
                                60.0, 290.0, 1);
                NeutralPaths rightSwoop = new NeutralPaths(this, autoAim, () -> AutoTrajectories.getRightCurve(), drive,
                                kicker,
                                300.0, 50.0, 1);

                NeutralPaths leftInOut = new NeutralPaths(this, autoAim, () -> AutoTrajectories.getLeftInOut(), drive,
                                kicker, 90.0, 90.0, 0);
                NeutralPaths rightInOut = new NeutralPaths(this, autoAim, () -> AutoTrajectories.getRightInOut(), drive,
                                kicker, 270.0, 270.0, 0);

                // Add steps to auto options

                Collections.addAll(orbitAuto, AutoSteps.FirstScore, AutoSteps.LeftSwoop, AutoSteps.LeftInOut,
                                AutoSteps.LeftTransition,
                                AutoSteps.SecondScore, AutoSteps.EmptyState);

                Collections.addAll(leftSwoopDepot, AutoSteps.FirstScore,
                                AutoSteps.LeftSwoop,
                                AutoSteps.LeftTransition,
                                AutoSteps.SecondScore,
                                AutoSteps.IntakeDepot,
                                AutoSteps.EmptyState);

                Collections.addAll(rightSwoopOutpost, AutoSteps.FirstScore,
                                AutoSteps.RightSwoop,
                                AutoSteps.RightTransition,
                                AutoSteps.EmptyState);

                Collections.addAll(leftDoubleSweep, AutoSteps.FirstScore, AutoSteps.LeftSwoop,
                                AutoSteps.LeftTransition, AutoSteps.SecondScore, AutoSteps.LeftTransition,
                                AutoSteps.LeftSwoop, AutoSteps.LeftTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Collections.addAll(rightDoubleSweep, AutoSteps.FirstScore, AutoSteps.RightSwoop,
                                AutoSteps.RightTransition, AutoSteps.SecondScore, AutoSteps.RightTransition,
                                AutoSteps.RightSwoop, AutoSteps.RightTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Collections.addAll(justShoot, AutoSteps.FirstScore, AutoSteps.EmptyState);

                Collections.addAll(shootDepot, AutoSteps.FirstScore, AutoSteps.IntakeDepot, AutoSteps.EmptyState);

                Collections.addAll(shootOutpost, AutoSteps.FirstScore, AutoSteps.IntakeOutpost, AutoSteps.EmptyState);

                Collections.addAll(leftInOutAuto, AutoSteps.FirstScore,
                                AutoSteps.LeftInOut,
                                AutoSteps.LeftTransition,
                                AutoSteps.SecondScore, AutoSteps.LeftTransition,
                                AutoSteps.FarLeftInOut, AutoSteps.LeftTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Collections.addAll(rightInOutAuto, AutoSteps.FirstScore,
                                AutoSteps.RightInOut,
                                AutoSteps.RightTransition, AutoSteps.SecondScore, AutoSteps.RightTransition,
                                AutoSteps.FarRightInOut, AutoSteps.RightTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                BooleanSupplier currentStateComplete = () -> {
                        if (currentStep.getCondition().getAsBoolean()) {
                                index = index + 1;
                                currentStep = Robot.autoSupplier.get().get(index);
                                return true;
                        } else {
                                return false;
                        }
                };

                auto.withChild(firstScore, () -> currentStep == AutoSteps.FirstScore, 0, "Auto to hub shot")
                                .withChild(secondScore, () -> currentStep == AutoSteps.SecondScore, 0,
                                                "Auto to second score")
                                .withChild(leftClimb, () -> currentStep == AutoSteps.LeftClimb, 0, "Auto to left climb")
                                .withChild(rightClimb, () -> currentStep == AutoSteps.RightClimb, 0,
                                                "Auto to right climb")
                                .withChild(leftSwoop,
                                                () -> currentStep == AutoSteps.LeftSwoop, 0,
                                                "Auto to left to right")
                                .withChild(rightSwoop, () -> currentStep == AutoSteps.RightSwoop, 0,
                                                "Auto to right to left")
                                .withChild(intakeDepot, () -> currentStep == AutoSteps.IntakeDepot, 0,
                                                "Auto to depot intake")
                                .withChild(intakeOutpost, () -> currentStep == AutoSteps.IntakeOutpost, 0,
                                                "Auto to outpost intake")
                                .withChild(leftTransition, () -> currentStep == AutoSteps.LeftTransition, 0,
                                                "Auto to left transition")
                                .withChild(rightTransition, () -> currentStep == AutoSteps.RightTransition, 0,
                                                "Auto to rightTransition")
                                .withChild(leftInOut, () -> currentStep == AutoSteps.LeftInOut, 0,
                                                "Auto to left in out")
                                .withChild(rightInOut, () -> currentStep == AutoSteps.RightInOut, 0,
                                                "Auto to right in out")
                                .withChild(emptyState, () -> currentStep == AutoSteps.EmptyState, 0,
                                                "Auto to empty state");

                leftClimb.withTransition(auto, currentStateComplete, 0, "Left climb to auto");
                rightClimb.withTransition(auto, currentStateComplete, "Right climb to auto");
                firstScore.withTransition(auto, currentStateComplete, 0, "Shoot hub to auto");
                secondScore.withTransition(auto, currentStateComplete, "Second score to auto");
                leftSwoop.withTransition(auto, currentStateComplete, 0, "Left to right to auto");
                rightSwoop.withTransition(auto, currentStateComplete, "Right to left to auto");
                intakeDepot.withTransition(auto, currentStateComplete, "Depot intake to auto");
                intakeOutpost.withTransition(auto, currentStateComplete, "Outpost intake to auto");
                leftTransition.withTransition(auto, currentStateComplete, 0, "Left transition to auto");
                rightTransition.withTransition(auto, currentStateComplete, 0, "Right transition to auto");
                leftInOut.withTransition(auto, currentStateComplete, 0, "Left in out to auto");
                rightInOut.withTransition(auto, currentStateComplete, 0, "Right in out to auto");
                emptyState.withTransition(auto, currentStateComplete, 0, "Empty to auto");

                // For SYSID (comment out for normal autos)
                // MAKE SURE YOU ADD AUTO TO THE REGISTER TO ROOT STATE

                // SysIDState auto = new SysIDState(this, chooser);
                // this.registerToRootState(auto);
                // teleop.withModeTransitions(disabled, teleop, auto, test);
                // test.withModeTransitions(disabled, teleop, auto, test);
                // disabled.withModeTransitions(disabled, teleop, auto, test);
                // auto.withModeTransitions(disabled, teleop, auto, test);
        }

        public void autoReset() {
                index = 0;
                currentStep = autoSupplier.get().get(index);
        }
}
