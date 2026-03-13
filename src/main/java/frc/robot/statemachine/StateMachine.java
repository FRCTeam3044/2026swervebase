package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.statemachine.States.Auto.AutoTrajectories;
import frc.robot.statemachine.States.Auto.EmptyState;
import frc.robot.statemachine.States.Auto.FarLeftInOut;
import frc.robot.statemachine.States.Auto.FarRightInOut;
import frc.robot.statemachine.States.Auto.IntakeAllianceZone;
import frc.robot.statemachine.States.Auto.IntakeDepot;
import frc.robot.statemachine.States.Auto.IntakeNeutralRight;
import frc.robot.statemachine.States.Auto.IntakeOutpost;
import frc.robot.statemachine.States.Auto.LeftFirstTransition;
import frc.robot.statemachine.States.Auto.LeftInOut;
import frc.robot.statemachine.States.Auto.IntakeNeutralLeft;
import frc.robot.statemachine.States.Auto.LeftTransition;
import frc.robot.statemachine.States.Auto.RightFirstTransition;
import frc.robot.statemachine.States.Auto.RightInOut;
import frc.robot.statemachine.States.Auto.RightTransition;
import frc.robot.statemachine.States.Auto.SecondShoot;
import frc.robot.statemachine.States.Auto.ShootToAlliedSide;
import frc.robot.statemachine.States.Auto.ShootToHub;
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
        public static ArrayList<AutoSteps> leftSwoop = new ArrayList<AutoSteps>();
        public static ArrayList<AutoSteps> rightSwoop = new ArrayList<AutoSteps>();

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
        private int index;

        private ConfigurableParameter<Boolean> forceEnableShooting = new ConfigurableParameter<>(false,
                        "Force Enable Shooting");

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
                AlliedZone alliedZone = new AlliedZone(this, driverController, autoAim);
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

                EmptyState emptyState = new EmptyState(this, drive);
                AutoClimb leftClimb = new AutoClimb(this, autoTargetUtil, AutoTargetUtil.getLeftTower(), autoAim, drive,
                                climber);
                AutoClimb rightClimb = new AutoClimb(this, autoTargetUtil, AutoTargetUtil.getRightTower(), autoAim,
                                drive,
                                climber);
                IntakeAllianceZone intakeAllianceZone = new IntakeAllianceZone(this, drive);
                IntakeNeutralLeft leftIntake = new IntakeNeutralLeft(this, autoTargetUtil,
                                AutoTrajectories::getLeftCurve, autoAim, drive, intake, kicker);

                IntakeNeutralRight rightIntake = new IntakeNeutralRight(this, autoTargetUtil,
                                AutoTrajectories::getRightCurve, autoAim, drive, intake, kicker);
                ShootToAlliedSide shootToAlliedSide = new ShootToAlliedSide(this, drive, autoAim);
                ShootToHub shootHub = new ShootToHub(this, autoTargetUtil, drive, intake, spindexer, kicker, turret,
                                hood, shooter,
                                autoAim);
                SecondShoot secondScore = new SecondShoot(this, autoTargetUtil, drive, intake, spindexer, kicker,
                                turret,
                                hood, shooter,
                                autoAim);
                LeftTransition leftTransition = new LeftTransition(this, drive);
                RightTransition rightTransition = new RightTransition(this, drive);
                LeftFirstTransition leftFirstTransition = new LeftFirstTransition(this, drive);
                RightFirstTransition rightFirstTransition = new RightFirstTransition(this, drive);
                IntakeDepot intakeDepot = new IntakeDepot(this, autoTargetUtil, autoAim, drive, intake, kicker,
                                shooter);
                IntakeOutpost intakeOutpost = new IntakeOutpost(this, autoTargetUtil, autoAim, drive, intake, kicker,
                                shooter);
                LeftInOut leftInOut = new LeftInOut(this, autoAim, drive, kicker);
                RightInOut rightInOut = new RightInOut(this, autoAim, drive, kicker);
                FarLeftInOut farLeftInOut = new FarLeftInOut(this, autoAim, drive,
                                kicker);
                FarRightInOut farRightInOut = new FarRightInOut(this, autoAim, drive,
                                kicker);

                Collections.addAll(leftSwoopDepot, AutoSteps.ShootToHub,
                                AutoSteps.LeftIntake,
                                AutoSteps.LeftTransition,
                                AutoSteps.SecondScore,
                                AutoSteps.IntakeDepot,
                                AutoSteps.EmptyState);

                Collections.addAll(rightSwoopOutpost, AutoSteps.ShootToHub,
                                AutoSteps.RightIntake,
                                AutoSteps.RightTransition,
                                AutoSteps.SecondScore,
                                AutoSteps.IntakeOutpost,
                                AutoSteps.EmptyState);

                Collections.addAll(leftDoubleSweep, AutoSteps.ShootToHub, AutoSteps.LeftIntake,
                                AutoSteps.LeftTransition, AutoSteps.SecondScore, AutoSteps.LeftTransition,
                                AutoSteps.LeftIntake, AutoSteps.LeftTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Collections.addAll(rightDoubleSweep, AutoSteps.ShootToHub, AutoSteps.RightIntake,
                                AutoSteps.RightTransition, AutoSteps.SecondScore, AutoSteps.RightTransition,
                                AutoSteps.RightIntake, AutoSteps.RightTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Collections.addAll(justShoot, AutoSteps.ShootToHub, AutoSteps.EmptyState);

                Collections.addAll(shootDepot, AutoSteps.ShootToHub, AutoSteps.IntakeDepot, AutoSteps.EmptyState);

                Collections.addAll(shootOutpost, AutoSteps.ShootToHub, AutoSteps.IntakeOutpost, AutoSteps.EmptyState);

                Collections.addAll(leftInOutAuto, AutoSteps.ShootToHub, AutoSteps.LeftFirstTransition,
                                AutoSteps.LeftInOut,
                                AutoSteps.LeftTransition,
                                AutoSteps.SecondScore, AutoSteps.LeftTransition,
                                AutoSteps.FarLeftInOut, AutoSteps.LeftTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Collections.addAll(rightInOutAuto, AutoSteps.ShootToHub, AutoSteps.RightFirstTransition,
                                AutoSteps.RightInOut,
                                AutoSteps.RightTransition, AutoSteps.SecondScore, AutoSteps.RightTransition,
                                AutoSteps.FarRightInOut, AutoSteps.RightTransition, AutoSteps.SecondScore,
                                AutoSteps.EmptyState);

                Supplier<ArrayList<AutoSteps>> autoSupplier = () -> {
                        if (autoChooser.get() == null) {
                                return leftSwoopDepot;
                        }
                        return autoChooser.get();
                };

                currentStep = autoSupplier.get().get(index);

                BooleanSupplier currentStateComplete = () -> {
                        if (currentStep.getCondition().getAsBoolean()) {
                                index = index + 1;
                                currentStep = autoSupplier.get().get(index);
                                return true;
                        } else {
                                return false;
                        }
                };

                auto.withChild(shootHub, () -> currentStep == AutoSteps.ShootToHub, 0, "Auto to hub shot")
                                .withChild(secondScore, () -> currentStep == AutoSteps.SecondScore, 0,
                                                "Auto to second score")
                                .withChild(leftClimb, () -> currentStep == AutoSteps.LeftClimb, 0, "Auto to left climb")
                                .withChild(rightClimb, () -> currentStep == AutoSteps.RightClimb, 0,
                                                "Auto to right climb")
                                .withChild(intakeAllianceZone, () -> currentStep == AutoSteps.IntakeAllianceZone, 0,
                                                "Auto to allied intake")
                                .withChild(leftIntake,
                                                () -> currentStep == AutoSteps.LeftIntake, 0,
                                                "Auto to left to right")
                                .withChild(rightIntake, () -> currentStep == AutoSteps.RightIntake, 0,
                                                "Auto to right to left")
                                .withChild(shootToAlliedSide, () -> currentStep == AutoSteps.ShootToAlliedSide, 0,
                                                "Auto to neutral shot")
                                .withChild(intakeDepot, () -> currentStep == AutoSteps.IntakeDepot, 0,
                                                "Auto to depot intake")
                                .withChild(intakeOutpost, () -> currentStep == AutoSteps.IntakeOutpost, 0,
                                                "Auto to outpost intake")
                                .withChild(leftTransition, () -> currentStep == AutoSteps.LeftTransition, 0,
                                                "Auto to left transition")
                                .withChild(rightTransition, () -> currentStep == AutoSteps.RightTransition, 0,
                                                "Auto to rightTransition")
                                .withChild(leftFirstTransition, () -> currentStep == AutoSteps.LeftFirstTransition, 0,
                                                "Auto to left first transition")
                                .withChild(rightFirstTransition, () -> currentStep == AutoSteps.RightFirstTransition, 0,
                                                "Auto to right first transition")
                                .withChild(leftInOut, () -> currentStep == AutoSteps.LeftInOut, 0,
                                                "Auto to left in out")
                                .withChild(rightInOut, () -> currentStep == AutoSteps.RightInOut, 0,
                                                "Auto to right in out")
                                .withChild(farLeftInOut, () -> currentStep == AutoSteps.FarLeftInOut, 0,
                                                "Auto to far left in out")
                                .withChild(farRightInOut, () -> currentStep == AutoSteps.FarRightInOut, 0,
                                                "Auto to far right in out")
                                .withChild(emptyState, () -> currentStep == AutoSteps.EmptyState, 0,
                                                "Auto to empty state");

                leftClimb.withTransition(auto, currentStateComplete, 0, "Left climb to auto");
                rightClimb.withTransition(auto, currentStateComplete, "Right climb to auto");
                shootHub.withTransition(auto, currentStateComplete, 0, "Shoot hub to auto");
                secondScore.withTransition(auto, currentStateComplete, "Second score to auto");
                shootToAlliedSide.withTransition(auto, currentStateComplete, 0, "Neutral shot to auto");
                intakeAllianceZone.withTransition(auto, currentStateComplete, 0, "Allied intake to auto");
                leftIntake.withTransition(auto, currentStateComplete, 0, "Left to right to auto");
                rightIntake.withTransition(auto, currentStateComplete, "Right to left to auto");
                intakeDepot.withTransition(auto, currentStateComplete, "Depot intake to auto");
                intakeOutpost.withTransition(auto, currentStateComplete, "Outpost intake to auto");
                leftTransition.withTransition(auto, currentStateComplete, 0, "Left transition to auto");
                rightTransition.withTransition(auto, currentStateComplete, 0, "Right transition to auto");
                leftFirstTransition.withTransition(auto, currentStateComplete, 0, "Left first transition to auto");
                rightFirstTransition.withTransition(auto, currentStateComplete, 0, "Right first transition to auto");
                leftInOut.withTransition(auto, currentStateComplete, 0, "Left in out to auto");
                rightInOut.withTransition(auto, currentStateComplete, 0, "Right in out to auto");
                farLeftInOut.withTransition(auto, currentStateComplete, 0, "Far left in out to auto");
                farRightInOut.withTransition(auto, currentStateComplete, 0, "Far right in out to auto");
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
}
