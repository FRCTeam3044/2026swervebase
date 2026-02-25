package frc.robot.statemachine.States;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
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
                        Turret turret, Hood hood, Kicker kicker, Spindexer spindexer, AutoTargetUtil autoTargetUtil) {
                super(stateMachine);

                SmartXboxController controller = new SmartXboxController(rawController, loop);

                Supplier<Pose2d> turretPoseSupplier = () -> Robot.robotContainer.drive.getPose()
                                .transformBy(ShotCalculator.robotToTurret.toTransform2d());
                Supplier<Pose3d> targetPoseSupplier = () -> {
                        if (AutoAimDataManager.mzCalibrationMode.get()) {
                                return autoTargetUtil.getAllianceZoneTarget();
                        } else {
                                return autoTargetUtil.getHub();
                        }
                };
                Supplier<Angle> turretAngleSupplier = () -> {
                        Pose2d turretPose = turretPoseSupplier.get();
                        Pose3d targetPose = targetPoseSupplier.get();
                        Translation2d targetTranslation = targetPose.getTranslation().toTranslation2d();
                        return targetTranslation.minus(turretPose.getTranslation()).getAngle().getMeasure();
                };
                controller.leftTrigger()
                                .whileTrue(Commands.parallel(
                                                shooter.runSpeed(() -> RPM.of(calibrationShotFlywheelSpeed.get())),
                                                turret.setAngle(() -> turretAngleSupplier.get()),
                                                hood.setPosition(() -> calibrationShotHoodPosition.get()))
                                                .withName("Calibration aiming"));
                controller.rightTrigger().whileTrue(kicker.shootKicker());
                controller.rightTrigger().whileFalse(kicker.blockKicker());
                startWhenActive(kicker.blockKicker().onlyWhile(controller.rightTrigger().negate())
                                .withName("Block Kicker"));
                startWhenActive(spindexer.setSpeed());

                controller.a().onTrue(Commands.runOnce(() -> {
                        Pose3d targetPose = targetPoseSupplier.get();
                        Translation2d target = targetPose.getTranslation().toTranslation2d();
                        Pose2d turretPosition = turretPoseSupplier.get();
                        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
                        ShotCalculator.dm.addShot(turretToTargetDistance, hood.getPosition(),
                                        shooter.getSpeed().in(RPM));
                }).withName("Record Shot"));
        }
}
