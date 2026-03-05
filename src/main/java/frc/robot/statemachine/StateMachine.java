package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.States.Auto.AutoClimb;
import frc.robot.statemachine.States.Auto.EmptyState;
import frc.robot.statemachine.States.Auto.IntakeAllianceZone;
import frc.robot.statemachine.States.Auto.IntakeNeutralZone;
import frc.robot.statemachine.States.Auto.ShootToAlliedSide;
import frc.robot.statemachine.States.Auto.ShootToHub;
import frc.robot.commands.DriveCommands;
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
import java.util.Collections;

import frc.robot.util.HubShiftUtil;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
        String autoWinner = DriverStation.getGameSpecificMessage();
        AllianceColor allianceColor = AllianceUtil.getAlliance();

        // Autos to choose from
        ArrayList<AutoSteps> testAutoRoutine = new ArrayList<AutoSteps>();

        // Trajectories to choose from
        ArrayList<Pose2d> leftToRightTraj = new ArrayList<Pose2d>();

        private AutoSteps currentStep;
        private int index;

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

                DisabledState disabled = new DisabledState(this, leds);
                currentState = disabled;
                State teleop = new TeleState(this, driverController, operatorController, drive, climber, intake,
                                spindexer, leds);
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
                AlliedZone alliedZone = new AlliedZone(this, autoAim);
                NeutralZone neutralZone = new NeutralZone(
                                this, driverController, operatorBoard, drive, intake, spindexer, kicker, turret, hood,
                                shooter,
                                autoAim);
                ActiveHub activeHub = new ActiveHub(
                                this, driverController, operatorController, drive, intake, spindexer, kicker, turret,
                                hood,
                                shooter,
                                autoAim);
                InactiveHub inactiveHub = new InactiveHub(this, driverController, drive, intake, spindexer, kicker,
                                turret,
                                hood);

                teleop.withDefaultChild(alliedZone).withChild(neutralZone);

                alliedZone.withChild(activeHub, () -> HubShiftUtil.getShiftedShiftInfo().active(), 0, "Active Hub");
                alliedZone.withChild(inactiveHub, () -> !HubShiftUtil.getShiftedShiftInfo().active(), 1,
                                "Inactive Hub");

                alliedZone.withTransition(
                                neutralZone, () -> autoTargetUtil.inNeutralZone(), 0, "Drive into Neutral Zone");
                neutralZone.withTransition(
                                alliedZone, () -> autoTargetUtil.inAllianceZone(), 0, "Drive into Allied Zone");

                activeHub.withTransition(
                                inactiveHub, () -> !HubShiftUtil.getShiftedShiftInfo().active(), 0,
                                "Hub becomes inactive");
                inactiveHub.withTransition(activeHub, () -> HubShiftUtil.getShiftedShiftInfo().active(), 0,
                                "Hub becomes active");

                teleop.withModeTransitions(disabled, teleop, auto, test);
                test.withModeTransitions(disabled, teleop, auto, test);
                auto.withModeTransitions(disabled, teleop, auto, test);
                disabled.withModeTransitions(disabled, teleop, auto, test);

                // Autonomous work

                Collections.addAll(leftToRightTraj, AutoTargetUtil.getRightNeutral(), AutoTargetUtil.getLeftNeutral());

                EmptyState emptyState = new EmptyState(this, drive);
                AutoClimb leftClimb = new AutoClimb(this, autoTargetUtil, AutoTargetUtil.getLeftTower(), autoAim, drive,
                                climber);
                AutoClimb rightClimb = new AutoClimb(this, autoTargetUtil, AutoTargetUtil.getRightTower(), autoAim,
                                drive,
                                climber);
                IntakeAllianceZone intakeAllianceZone = new IntakeAllianceZone(this, drive);
                IntakeNeutralZone intakeNeutralZone = new IntakeNeutralZone(this, leftToRightTraj, drive, intake);
                ShootToAlliedSide shootToAlliedSide = new ShootToAlliedSide(this, drive, autoAim);
                ShootToHub shootHub = new ShootToHub(this, autoTargetUtil, drive, intake, spindexer, kicker, turret,
                                hood, shooter,
                                autoAim, climber);

                Collections.addAll(testAutoRoutine, AutoSteps.IntakeNeutralZone,
                                AutoSteps.EmptyState);
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

                auto.withChild(shootHub, () -> currentStep == AutoSteps.ShootToHub, 0, "Auto to hub shot")
                                .withChild(leftClimb, () -> currentStep == AutoSteps.LeftClimb, 0, "Auto to left climb")
                                .withChild(rightClimb, () -> currentStep == AutoSteps.RightClimb, 0,
                                                "Auto to right climb")
                                .withChild(intakeAllianceZone, () -> currentStep == AutoSteps.IntakeAllianceZone, 0,
                                                "Auto to allied intake")
                                .withChild(intakeNeutralZone,
                                                () -> currentStep == AutoSteps.IntakeNeutralZone, 0,
                                                "Auto to neutral intake")
                                .withChild(shootToAlliedSide, () -> currentStep == AutoSteps.ShootToAlliedSide, 0,
                                                "Auto to neutral shot")
                                .withChild(emptyState, () -> currentStep == AutoSteps.EmptyState, 0,
                                                "Auto to empty state");

                leftClimb.withTransition(auto, currentStateComplete, 0, "Left climb to auto");
                rightClimb.withTransition(auto, currentStateComplete, "Right climb to auto");
                shootHub.withTransition(auto, currentStateComplete, 0, "Shoot hub to auto");
                shootToAlliedSide.withTransition(auto, currentStateComplete, 0, "Neutral shot to auto");
                intakeAllianceZone.withTransition(auto, currentStateComplete, 0, "Allied intake to shot");
                intakeNeutralZone.withTransition(auto, currentStateComplete, 0, "Neutral intake to shot");
                emptyState.withTransition(auto, currentStateComplete, 0, "Empty to auto");

                // For SYSID (comment out for normal autos)
                // MAKE SURE YOU ADD AUTO TO THE REGISTER TO ROOT STATE

                // SysIDState auto = new SysIDState(this, chooser);
                // teleop.withModeTransitions(disabled, teleop, auto, test);
                // test.withModeTransitions(disabled, teleop, auto, test);
                // disabled.withModeTransitions(disabled, teleop, auto, test);
                // auto.withModeTransitions(disabled, teleop, auto, test);
        }
}
