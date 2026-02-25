// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int pigeonCanId = 9;

    public static final int frontLeftDriveCanId = 13;
    public static final int backLeftDriveCanId = 16;
    public static final int frontRightDriveCanId = 12;
    public static final int backRightDriveCanId = 17;

    public static final int frontLeftTurnCanId = 8;
    public static final int backLeftTurnCanId = 5;
    public static final int frontRightTurnCanId = 1;
    public static final int backRightTurnCanId = 2;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth
    // and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;

    public static final PIDController xController = new ConfigurablePIDController(1, 0, 0,
            "Pathfinding X Controller");
    public static final PIDController yController = new ConfigurablePIDController(1, 0, 0,
            "Pathfinding Y Controller");

    public static final ProfiledPIDController angleController = new ConfigurableProfiledPIDController(
            6.0,
            0,
            0,
            // new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond.get(),
            // kMaxAngularAccelerationRadiansPerSecondSquared.get()),
            new TrapezoidProfile.Constraints(8, 20),
            "Pathfinding Theta Controller");

    public static final HolonomicDriveController driveController = new HolonomicDriveController(
            xController, yController, angleController);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withBumperSize(Inches.of(30), Inches.of(30))
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(
                    new SwerveModuleSimulationConfig(
                            driveGearbox,
                            turnGearbox,
                            driveMotorReduction,
                            turnMotorReduction,
                            Volts.of(0.1),
                            Volts.of(0.1),
                            Meters.of(wheelRadiusMeters),
                            KilogramSquareMeters.of(0.02),
                            wheelCOF));

    public static final Distance bumperSize = Inches.of(36.125);
    public static final Distance mapleBumperSize = Constants.currentMode == Constants.Mode.SIM ? Inches.of(36.25)
            : bumperSize;

    public static final Pathfinder pathfinder = (new PathfinderBuilder(Field.REBUILT_2026_TRENCH))
            .setNormalizeCorners(false)
            .setCornerDist(1)
            .setCornerPointSpacing(0.06)
            .setRobotLength(mapleBumperSize.in(Meters) + 0.35)
            .setRobotWidth(mapleBumperSize.in(Meters) + 0.35)
            .build();
}
