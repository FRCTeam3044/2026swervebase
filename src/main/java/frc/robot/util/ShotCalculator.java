// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({ GeomUtil.class })
public class ShotCalculator {
    private static ShotCalculator instance;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle;
    private Rotation2d turretAngle;
    private double hoodAngle = Double.NaN;
    private double turretVelocity;
    private double hoodVelocity;

    public static ShotCalculator getInstance() {
        if (instance == null)
            instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d turretAngle,
            double turretVelocity,
            double hoodAngle,
            double hoodVelocity,
            double flywheelSpeed) {
    }

    // Cache parameters
    private ShootingParameters latestParameters = null;

    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;
    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static Transform3d robotToTurret = new Transform3d(0, 0, 0.381, Rotation3d.kZero);

    static {
        minDistance = 1.01;
        maxDistance = 5.598;
        phaseDelay = 0.03;

        double[] distances = new double[] { 1.01, 1.665, 2.1307, 3.0156, 4.4437, 5.598 };

        shotHoodAngleMap.put(distances[0], Rotation2d.fromDegrees(82.0));
        shotHoodAngleMap.put(distances[1], Rotation2d.fromDegrees(76.0));
        shotHoodAngleMap.put(distances[2], Rotation2d.fromDegrees(72.0));
        shotHoodAngleMap.put(distances[3], Rotation2d.fromDegrees(65.0));
        shotHoodAngleMap.put(distances[4], Rotation2d.fromDegrees(55.0));
        shotHoodAngleMap.put(distances[5], Rotation2d.fromDegrees(55.0));

        shotFlywheelSpeedMap.put(distances[0], 7.0);
        shotFlywheelSpeedMap.put(distances[1], 7.0);
        shotFlywheelSpeedMap.put(distances[2], 7.1);
        shotFlywheelSpeedMap.put(distances[3], 7.5);
        shotFlywheelSpeedMap.put(distances[4], 8.2);
        shotFlywheelSpeedMap.put(distances[5], 9.0);

        timeOfFlightMap.put(distances[0], 1.0);
        timeOfFlightMap.put(distances[1], 0.98);
        timeOfFlightMap.put(distances[2], 0.9605);
        timeOfFlightMap.put(distances[3], 0.98);
        timeOfFlightMap.put(distances[4], 0.96);
        timeOfFlightMap.put(distances[5], 1.119);
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) {
            return latestParameters;
        }

        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = Robot.robotContainer.drive.getPose();
        ChassisSpeeds robotRelativeVelocity = Robot.robotContainer.drive.getRobotRelativeChassisSpeeds();
        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        // Translation2d target =
        // AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Translation2d target = Robot.robotContainer.autoTargetUtil.getHub().getTranslation().toTranslation2d();
        Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = Robot.robotContainer.drive.getFieldRelativeChassisSpeeds();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getY() * Math.cos(robotAngle)
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
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getDegrees();
        if (lastTurretAngle == null)
            lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle))
            lastHoodAngle = hoodAngle;
        turretVelocity = turretAngleFilter.calculate(
                turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
        hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;
        latestParameters = new ShootingParameters(
                lookaheadTurretToTargetDistance >= minDistance
                        && lookaheadTurretToTargetDistance <= maxDistance,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        // Log calculated values
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
        Logger.recordOutput("ShotCalculator/HoodAngle", hoodAngle);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
