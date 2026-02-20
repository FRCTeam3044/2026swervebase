package frc.robot.statemachine.States;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AutoAimDataManager;
import frc.robot.util.AutoTargetUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.ShotCalculator;
import lombok.experimental.ExtensionMethod;
import me.nabdev.oxconfig.ConfigurableClass;
import me.nabdev.oxconfig.ConfigurableClassParam;
import me.nabdev.oxconfig.OxConfig;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

@ExtensionMethod({ GeomUtil.class })
public class CalibrationState extends State implements ConfigurableClass {
        private ConfigurableClassParam<Double> calibrationShotFlywheelSpeed = new ConfigurableClassParam<>(this, 3000.0,
                        "CalibrationShotFlywheelSpeed");
        private ConfigurableClassParam<Double> calibrationShotHoodPosition = new ConfigurableClassParam<>(this, 0.5,
                        "CalibrationShotHoodPosition");
        private ConfigurableClassParam<Double> calibrationShotTurretAngle = new ConfigurableClassParam<>(this, 0.0,
                        "CalibrationShotTurretAngle");

        private final List<ConfigurableClassParam<?>> parameters = new ArrayList<>(
                        Arrays.asList(calibrationShotFlywheelSpeed, calibrationShotHoodPosition,
                                        calibrationShotTurretAngle));

        public CalibrationState(StateMachineBase stateMachine, CommandXboxController rawController, Drive drive,
                        Shooter shooter,
                        Turret turret, Hood hood, Kicker kicker, Spindexer spindexer, AutoTargetUtil autoTargetUtil) {
                super(stateMachine);

                SmartXboxController controller = new SmartXboxController(rawController, loop);
                controller.leftTrigger()
                                .whileTrue(Commands.parallel(
                                                shooter.runSpeed(() -> RPM.of(calibrationShotFlywheelSpeed.get())),
                                                turret.setAngle(() -> Degrees.of(calibrationShotTurretAngle.get())),
                                                hood.setPosition(() -> calibrationShotHoodPosition.get()))
                                                .withName("Calibration aiming"));
                controller.rightTrigger().whileTrue(kicker.shootKicker());
                controller.rightTrigger().whileFalse(kicker.blockKicker());
                startWhenActive(kicker.blockKicker().onlyWhile(controller.rightTrigger().negate())
                                .withName("Block Kicker"));
                startWhenActive(spindexer.setSpeed());

                controller.a().onTrue(Commands.runOnce(() -> {
                        Pose2d estimatedPose = Robot.robotContainer.drive.getPose();

                        Pose3d targetPose;

                        if (AutoAimDataManager.mzCalibrationMode.get()) {
                                targetPose = autoTargetUtil.getAllianceZoneTarget();
                        } else {
                                targetPose = autoTargetUtil.getHub();
                        }

                        Translation2d target = targetPose.getTranslation().toTranslation2d();
                        Pose2d turretPosition = estimatedPose.transformBy(ShotCalculator.robotToTurret.toTransform2d());
                        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
                        ShotCalculator.dm.addShot(turretToTargetDistance, hood.getPosition(),
                                        shooter.getSpeed().in(RPM));
                }).withName("Record Shot"));

                OxConfig.registerConfigurableClass(this);
        }

        @Override
        public List<ConfigurableClassParam<?>> getParameters() {
                return parameters;
        }

        @Override
        public String getKey() {
                return "CalibrationState";
        }
}
