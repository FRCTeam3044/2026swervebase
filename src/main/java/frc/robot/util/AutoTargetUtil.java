package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil.AllianceColor;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vertex;

public class AutoTargetUtil {
  private final Drive drive;

  final double blueSideLine = 4.57482575;
  final double redSideLine = 11.96622575;

  private Pose3d hub = new Pose3d(new Translation3d(4.62, 8.069 / 2.0, 2), new Rotation3d());
  private Pose3d outpostAllianceTarget = new Pose3d(new Translation3d(3.3, 1.7, 2), new Rotation3d());
  private Pose3d depotAllianceTarget = new Pose3d(new Translation3d(3.3, 6.3, 2), new Rotation3d());

  private static Pose2d testNeutralZonePosition = new Pose2d(7, 4, Rotation2d.fromDegrees(0));

  private static Pose2d leftTower = new Pose2d(1.1445875, 3.26602725, Rotation2d.fromDegrees(0));
  private static Pose2d middleTower = new Pose2d(1.1445875, 3.74570625, Rotation2d.fromDegrees(0));
  private static Pose2d rightTower = new Pose2d(1.1445875, 4.22538525, Rotation2d.fromDegrees(0));

  private static Pose2d rightNeutral = new Pose2d(9.5, 6, Rotation2d.fromDegrees(60));
  private static Pose2d leftNeutral = new Pose2d(9.5, 1.5, Rotation2d.fromDegrees(60));

  private static Obstacle redAllianceZone = Obstacle.createObstacle(
      new Vertex(12.5, 0),
      new Vertex(12.5, 8.0692625),
      new Vertex(16.5410515, 8.0692625),
      new Vertex(16.5410515, 0));

  private static Obstacle blueAllianceZone = Obstacle.createObstacle(
      new Vertex(0, 0),
      new Vertex(0, 8.0692625),
      new Vertex(4, 8.0692625),
      new Vertex(4, 0));

  public AutoTargetUtil(Drive drive) {
    this.drive = drive;
  }

  public Pose3d getHub() {
    return AllianceUtil.getPose3dForAlliance(hub);
  }

  public static Pose2d getRightNeutral() {
    return AllianceUtil.getPoseForAlliance(rightNeutral);
  }

  public static Pose2d getLeftNeutral() {
    return AllianceUtil.getPoseForAlliance(leftNeutral);
  }

  public static Pose2d getNeutralZone() {
    return AllianceUtil.getPoseForAlliance(testNeutralZonePosition);
  }

  public static Pose2d getLeftTower() {
    return AllianceUtil.getPoseForAlliance(leftTower);
  }

  public static Obstacle allianceSide() {
    if (AllianceUtil.getAlliance() == AllianceColor.BLUE) {
      return blueAllianceZone;
    } else {
      return redAllianceZone;
    }
  }

  public Pose3d getAllianceZoneTarget() {
    // pick whichever target is closer
    Pose3d depotTarget = AllianceUtil.getPose3dForAlliance(depotAllianceTarget);
    Pose3d outpostTarget = AllianceUtil.getPose3dForAlliance(outpostAllianceTarget);
    if (drive.getPose().getTranslation().getDistance(depotTarget.getTranslation().toTranslation2d()) < drive
        .getPose().getTranslation().getDistance(outpostTarget.getTranslation().toTranslation2d())) {
      return depotTarget;
    } else {
      return outpostTarget;
    }
  }

  public boolean inNeutralZone() {
    if (drive.getPose().getX() < redSideLine && drive.getPose().getX() > blueSideLine) {
      return true;
    }
    return false;
  }

  public boolean inAllianceZone() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();

    if ((drive.getPose().getX() > redSideLine
        && (allianceColor == AllianceColor.RED || allianceColor == AllianceColor.UNKNOWN))
        || (drive.getPose().getX() < blueSideLine
            && (allianceColor == AllianceColor.BLUE || allianceColor == AllianceColor.UNKNOWN))) {
      return true;
    }
    return false;
  }

  public boolean inOpponentZone() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();
    if ((drive.getPose().getX() > redSideLine && allianceColor == AllianceColor.BLUE)
        || (drive.getPose().getX() < blueSideLine && allianceColor == AllianceColor.RED)) {
      return true;
    }
    return false;
  }
}
