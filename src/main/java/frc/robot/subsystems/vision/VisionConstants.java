// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class VisionConstants {
        private final static ConfigurableParameter<Boolean> onlyHubTagLayout = new ConfigurableParameter<>(
                        false, "Only Hub Tag Layout");

        // AprilTag layout
        // public static AprilTagFieldLayout aprilTagLayout =
        // AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static AprilTagFieldLayout aprilTagLayout;
        public static AprilTagFieldLayout allTags;
        public static AprilTagFieldLayout onlyHubTags;

        static {
                try {
                        allTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                        onlyHubTags = new AprilTagFieldLayout(
                                        Filesystem.getDeployDirectory() + "/AprilTagLayouts/2026-rebuilt-onlyhub.json");
                } catch (Exception e) {
                        e.printStackTrace();
                }
        }

        public static AprilTagFieldLayout getAprilTagLayout() {
                return onlyHubTagLayout.get() ? onlyHubTags : allTags;
        }

        // Camera names, must match names configured on coprocessor
        public static String fsCamName = "fore_star";
        public static String ssCamName = "sensor_star";
        public static String spCamName = "sensor_port";
        public static String fpCamName = "fore_port";

        // Robot to camera transforms
        // Port (left)
        public static Transform3d robotToSp = new Transform3d(
                        Units.inchesToMeters(
                                        -0.75),
                        Units.inchesToMeters(12.746915),
                        Units.inchesToMeters(20.750000),
                        new Rotation3d(
                                        Units.degreesToRadians(0.0),
                                        Units.degreesToRadians(0.0),
                                        Units.degreesToRadians(116.352749)));

        // Starboard (right)
        public static Transform3d robotToSs = new Transform3d(
                        Units.inchesToMeters(
                                        -2.75),
                        Units.inchesToMeters(-12.746915),
                        Units.inchesToMeters(20.750000),
                        new Rotation3d(
                                        Units.degreesToRadians(0.0),
                                        Units.degreesToRadians(0.0),
                                        Units.degreesToRadians(244.65)));

        // Fore-starboard (back right)
        public static Transform3d robotToFs = new Transform3d(
                        Units.inchesToMeters(
                                        10.75),
                        Units.inchesToMeters(-10.25),
                        Units.inchesToMeters(9),
                        new Rotation3d(
                                        Units.degreesToRadians(0.0),
                                        Units.degreesToRadians(-25.0),
                                        Units.degreesToRadians(315.0)));

        // Fore-port (back left)
        public static Transform3d robotToFp = new Transform3d(
                        Units.inchesToMeters(10.75),
                        Units.inchesToMeters(
                                        10.25),
                        Units.inchesToMeters(9),
                        new Rotation3d(
                                        Units.degreesToRadians(0.0),
                                        Units.degreesToRadians(-10.0),
                                        Units.degreesToRadians(45.0)));

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = 0.04; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] cameraStdDevFactors = new double[] {
                        1.5, // Fs
                        2.5, // SS
                        2.5, // SP
                        1.5 // FP
        };

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
