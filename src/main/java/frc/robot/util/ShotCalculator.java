// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import lombok.experimental.ExtensionMethod;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.io.FileNotFoundException;

import org.littletonrobotics.junction.Logger;

@ExtensionMethod({ GeomUtil.class })
public class ShotCalculator {
    private static ShotCalculator instance;

    private Rotation2d turretAngle;
    private double hoodPosition = Double.NaN;

    public static ShotCalculator getInstance() {
        if (instance == null)
            instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isTurretValid,
            boolean isDistanceValid,
            Rotation2d turretAngle,
            double hoodPosition,
            double flywheelSpeed) {
    }

    // Cache parameters
    private ShootingParameters latestParameters = null;

    public static AutoAimDataManager dm;

    private static ConfigurableParameter<Double> phaseDelay = new ConfigurableParameter<Double>(0.05, "Phase Delay");
    private static ConfigurableParameter<Double> rotPhaseDelay = new ConfigurableParameter<Double>(0.05,
            "Rotation Phase Delay");
    private static ConfigurableParameter<Double> maxDistance = new ConfigurableParameter<Double>(5.5, "Max Distance");
    private static ConfigurableParameter<Double> hoodAccounting = new ConfigurableParameter<Double>(0.001,
            "Hood Account");
    private static ConfigurableParameter<Double> turretFudge = new ConfigurableParameter<Double>(0.01,
            "Turret Fudge");

    private ConfigurableParameter<Double> safeHubShootTolerance = new ConfigurableParameter<>(2.0,
            "SafeShootNetTolerance");

    private ConfigurableParameter<Double> safeNeutralShootTolerance = new ConfigurableParameter<>(2.5,
            "SafeShootNeutralTolerance");

    private LinearFilter flywheelAvg = LinearFilter.movingAverage(5);
    private LinearFilter velFilter = LinearFilter.movingAverage(5);

    public static Transform3d robotToTurret = new Transform3d(Units.inchesToMeters(6.75), Units.inchesToMeters(3.75),
            Units.inchesToMeters(15), Rotation3d.kZero);

    static {
        try {
            dm = new AutoAimDataManager();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static void periodic() {
        dm.periodic();
    }

    public ShootingParameters getParameters(Pose3d targetPose, boolean secondaryValues) {
        if (latestParameters != null) {
            return latestParameters;
        }

        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = Robot.robotContainer.drive.getPose();
        ChassisSpeeds robotRelativeVelocity = Robot.robotContainer.drive.getRobotRelativeChassisSpeeds();
        // robotRelativeVelocity.omegaRadiansPerSecond *= -1;
        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        robotRelativeVelocity.vxMetersPerSecond * phaseDelay.get(),
                        robotRelativeVelocity.vyMetersPerSecond * phaseDelay.get(),
                        robotRelativeVelocity.omegaRadiansPerSecond * rotPhaseDelay.get()));

        // Calculate distance from turret to target
        // Translation2d target =
        // AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Translation2d target = targetPose.getTranslation().toTranslation2d();
        Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = Robot.robotContainer.drive.getFieldRelativeChassisSpeeds();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getY() * -Math.cos(robotAngle)
                                - robotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getX() * Math.cos(robotAngle)
                                - robotToTurret.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = dm.getTimeOfFlightMap(secondaryValues).get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        turretAngle = new Rotation2d(target.minus(lookaheadPose.getTranslation()).getAngle().getMeasure()
                .minus(RobotContainer.getInstance().drive.getPose().getRotation().getMeasure()));
        hoodPosition = dm.getShotHoodPositionMap(secondaryValues).get(lookaheadTurretToTargetDistance);

        double targetFlywheelSpeed = dm.getShotFlywheelSpeedMap(secondaryValues).get(lookaheadTurretToTargetDistance);
        double flywheelSpeed = flywheelAvg.calculate(RobotContainer.getInstance().shooter.getSpeed());
        double hoodAdjustment = -(flywheelSpeed - targetFlywheelSpeed) * hoodAccounting.get();
        double turretAdjustment = velFilter.calculate(robotVelocity.omegaRadiansPerSecond) * turretFudge.get();

        hoodPosition += hoodAdjustment;
        turretAngle = turretAngle.plus(Rotation2d.fromRadians(turretAdjustment));

        double acceptableAngle = Math
                .atan2(RobotContainer.getInstance().autoTargetUtil.inAllianceZone() ? safeHubShootTolerance.get()
                        : safeNeutralShootTolerance.get(), lookaheadTurretToTargetDistance);
        Logger.recordOutput("ShotCalculator/AcceptableAngle", acceptableAngle);
        boolean isGood = RobotContainer.getInstance().turret.isAtTarget(Math.toDegrees(acceptableAngle));

        Logger.recordOutput("ShotCalculator/TurretGood", isGood);

        latestParameters = new ShootingParameters(
                isGood,
                lookaheadTurretToTargetDistance >= dm.getMinDistance(secondaryValues)
                        && lookaheadTurretToTargetDistance <= maxDistance.get(),
                turretAngle,
                hoodPosition, targetFlywheelSpeed);

        // Log calculated values
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
        Logger.recordOutput("ShotCalculator/hoodPosition", hoodPosition);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
