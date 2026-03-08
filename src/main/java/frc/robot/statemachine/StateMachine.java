package frc.robot.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.States.CalibrationState;
import frc.robot.statemachine.States.DisabledState;
import frc.robot.statemachine.States.NormalTestState;
import frc.robot.statemachine.States.SysIDState;
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
import frc.robot.util.HubShiftUtil;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
        String autoWinner = DriverStation.getGameSpecificMessage();
        AllianceColor allianceColor = AllianceUtil.getAlliance();

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
                        LoggedDashboardChooser<Command> chooser) {
                super();

                DisabledState disabled = new DisabledState(this, leds);
                currentState = disabled;
                State teleop = new TeleState(this, driverController, operatorController, drive, climber, intake,
                                spindexer, hood, turret, leds);
                State test = new TestState(this, hood, turret);

                this.registerToRootState(test, teleop, disabled);
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

                // teleop.withModeTransitions(disabled, teleop, test);
                // test.withModeTransitions(disabled, teleop, test);
                // disabled.withModeTransitions(disabled, teleop, test);

                // For SYSID (comment out for normal autos)
                // MAKE SURE YOU ADD AUTO TO THE REGISTER TO ROOT STATE

                SysIDState auto = new SysIDState(this, chooser);
                this.registerToRootState(auto);
                teleop.withModeTransitions(disabled, teleop, auto, test);
                test.withModeTransitions(disabled, teleop, auto, test);
                disabled.withModeTransitions(disabled, teleop, auto, test);
                auto.withModeTransitions(disabled, teleop, auto, test);
        }
}
