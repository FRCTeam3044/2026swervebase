package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.Drive;

public class AutoTargetUtils {
  private Drive drive;
  private AutoAim autoAim;
  private Pose3d target = new Pose3d(new Translation3d(4.62, 8.069 / 2.0, 2), new Rotation3d());

  public AutoTargetUtils(Drive drive, AutoAim autoAim) {
    this.drive = drive;
    this.autoAim = autoAim;
  }

  public Pose3d getTarget() {
    return target;
  }

  public Pose3d getEffectiveTarget() {
    return ShootOnFlyCalculator.calculateEffectiveTargetLocation(
        drive.getPose(),
        target,
        drive.getFieldRelativeChassisSpeeds(),
        drive.getChassisAccelerations(),
        (dist) -> {
          double speed =
              autoAim.calculateSpeed(dist); // * Math.cos(autoAim.calculateAngle(dist) * Math.PI /
          // 180);

          // System.out.println(
          // "Distance: "
          // + dist
          // + " Speed: "
          // + speed
          // + " Total Speed: "
          // + autoAim.calculateSpeed(dist)
          // + " Angle: "
          // + autoAim.calculateAngle(dist));

          return speed;
        },
        100,
        0);
  }
}
