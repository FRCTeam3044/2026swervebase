package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.statemachine.States.Auto.AutoClimb;
import frc.robot.statemachine.States.Auto.IntakeAllianceZone;
import frc.robot.statemachine.States.Auto.IntakeNeutralZone;
import frc.robot.statemachine.States.Auto.ShootToAlliedSide;
import frc.robot.statemachine.States.Auto.ShootToHub;
import frc.robot.statemachine.States.AutoState;
import frc.robot.statemachine.States.DisabledState;
import frc.robot.statemachine.States.Tele.ActiveHub;
import frc.robot.statemachine.States.Tele.AlliedZone;
import frc.robot.statemachine.States.Tele.InactiveHub;
import frc.robot.statemachine.States.Tele.NeutralZone;
import frc.robot.statemachine.States.TeleState;
import frc.robot.statemachine.States.TestState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AllianceUtil.AllianceColor;
import frc.robot.util.AutoEnums.AutoSteps;
import frc.robot.util.AutoTargetUtil;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
        String autoWinner = DriverStation.getGameSpecificMessage();
        AllianceColor allianceColor = AllianceUtil.getAlliance();

        public StateMachine(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        Drive drive,
                        Intake intake,
                        Spindexer spindexer,
                        Kicker kicker,
                        Shooter shooter,
                        Turret turret,
                        Hood hood,
                        AutoTargetUtil autoTargetUtil) {
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

                State disabled = new DisabledState(this);
                currentState = disabled;
                State teleop = new TeleState(this, driverController, drive);
                State test = new TestState(
                                this, driverController, operatorController, drive, hood, intake, kicker, shooter,
                                spindexer, turret);
                State auto = new AutoState(this);

                this.registerToRootState(test, teleop, disabled, auto);

                // Teleop States
                AlliedZone alliedZone = new AlliedZone(this);
                NeutralZone neutralZone = new NeutralZone(
                                this, driverController, drive, intake, spindexer, kicker, turret, hood, shooter);
                ActiveHub activeHub = new ActiveHub(
                                this, driverController, drive, intake, spindexer, kicker, turret, hood, shooter);
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

                AutoClimb autoClimb = new AutoClimb(this);
                IntakeAllianceZone intakeAllianceZone = new IntakeAllianceZone(this);
                IntakeNeutralZone intakeNeutralZone = new IntakeNeutralZone(this, intake);
                ShootToAlliedSide shootToAlliedSide = new ShootToAlliedSide(this);
                ShootToHub shootHub = new ShootToHub(this, drive, intake, spindexer, kicker, turret, hood, shooter);

                auto.withDefaultChild(shootHub)
                                .withChild(autoClimb)
                                .withChild(intakeAllianceZone)
                                .withChild(intakeNeutralZone)
                                .withChild(shootToAlliedSide);

                ArrayList<AutoSteps> testAutoRoutine = new ArrayList<>();
                testAutoRoutine.add(AutoSteps.ShootToHub);
                testAutoRoutine.add(AutoSteps.IntakeNeutralZone);
                testAutoRoutine.add(AutoSteps.ShootToHub);
                testAutoRoutine.add(AutoSteps.AutoClimb);
                AutoSteps currentStep;

                // TODO: Add "completed" boolean suppliers
                autoClimb.withTransition(auto, () -> false, 0, "Climb to auto");
                shootHub.withTransition(auto, () -> false, 0, "Shoot hub to auto");
                shootToAlliedSide.withTransition(auto, () -> false, 0, "Neutral shot to auto");
                intakeAllianceZone.withTransition(auto, () -> false, 0, "Allied intake to shot");
                intakeNeutralZone.withTransition(auto, () -> false, 0, "Neutral intake to shot");

                // TODO: Add "next in line" boolean suppliers
                auto.withTransition(autoClimb, () -> false, 0, "Auto to climb");
                auto.withTransition(shootHub, () -> false, 0, "Auto to hub shot");
                auto.withTransition(shootToAlliedSide, () -> false, 0, "Auto to neutral shot");
                auto.withTransition(intakeAllianceZone, () -> false, 0, "Auto to allied intake");
                auto.withTransition(intakeNeutralZone, () -> false, 0, "Auto to neutral intake");
        }
}
