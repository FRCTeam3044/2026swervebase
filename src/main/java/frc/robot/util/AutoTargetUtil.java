package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil.AllianceColor;

public class AutoTargetUtil {
  private final Drive drive;

  final double blueSideLine = 4.57482575;
  final double redSideLine = 11.96622575;

  private Pose3d hub = new Pose3d(new Translation3d(4.62, 8.069 / 2.0, 2), new Rotation3d());
  private Pose3d outpostAllianceTarget = new Pose3d(new Translation3d(3.3, 1.7, 2), new Rotation3d());
  private Pose3d depotAllianceTarget = new Pose3d(new Translation3d(3.3, 6.3, 2), new Rotation3d());

  public AutoTargetUtil(Drive drive) {
    this.drive = drive;
  }

  public Pose3d getHub() {
    return AllianceUtil.getPose3dForAlliance(hub);
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
