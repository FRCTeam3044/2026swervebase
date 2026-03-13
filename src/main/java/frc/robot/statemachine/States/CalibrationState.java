package frc.robot.statemachine.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AutoAimDataManager;
import frc.robot.util.AutoTargetUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.ShotCalculator;
import lombok.experimental.ExtensionMethod;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

@ExtensionMethod({ GeomUtil.class })
public class CalibrationState extends State {
        private ConfigurableParameter<Double> calibrationShotFlywheelSpeed = new ConfigurableParameter<>(3000.0,
                        "CalibrationShotFlywheelSpeed");
        private ConfigurableParameter<Double> calibrationShotHoodPosition = new ConfigurableParameter<>(0.5,
                        "CalibrationShotHoodPosition");

        public CalibrationState(StateMachineBase stateMachine, CommandXboxController rawController, Drive drive,
                        Shooter shooter,
                        Turret turret, Hood hood, Kicker kicker, Spindexer spindexer, Intake intake,
                        AutoTargetUtil autoTargetUtil) {
                super(stateMachine);

                SmartXboxController controller = new SmartXboxController(rawController, loop);

                controller.leftTrigger()
                                .whileTrue(Commands.parallel(
                                                shooter.runSpeed(() -> calibrationShotFlywheelSpeed.get()),
                                                turret.setAngle(() -> ShotCalculator.dm.getTurretAngle(
                                                                AutoAimDataManager.mzCalibrationMode.get())),
                                                hood.setPosition(() -> calibrationShotHoodPosition.get()))
                                                .withName("Calibration aiming"));
                controller.rightTrigger().whileTrue(kicker.shootKicker());
                // controller.rightTrigger().whileFalse(kicker.blockKicker());
                // startWhenActive(kicker.blockKicker().onlyWhile(controller.rightTrigger().negate())
                // .withName("Block Kicker"));
                startWhenActive(spindexer.setSpeed());
                startWhenActive(intake.runRollers());
                startWhenActive(intake.intakeBottom());
                startWhenActive(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -rawController.getLeftY(),
                                                () -> -rawController.getLeftX(),
                                                () -> -rawController.getRightX(),
                                                true, () -> false));

                controller.rightTrigger().onTrue(Commands.runOnce(() -> {
                        double turretToTargetDistance = AutoAimDataManager.mzCalibrationMode.get()
                                        ? ShotCalculator.dm.getDistToAz()
                                        : ShotCalculator.dm.getDistToHub();
                        ShotCalculator.dm.addShot(turretToTargetDistance, hood.getPosition(),
                                        shooter.getSpeed());
                }).withName("Record Shot"));
        }
}
