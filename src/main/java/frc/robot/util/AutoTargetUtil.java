package frc.robot.util;

import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil.AllianceColor;

public class AutoTargetUtil {
  private final Drive drive;

  final double redSideLine = 4.57482575;
  final double blueSideLine = 11.96622575;

  public AutoTargetUtil(Drive drive) {
    this.drive = drive;
  }

  public boolean inNeutralZone() {
    if (drive.getPose().getX() > redSideLine && drive.getPose().getX() < blueSideLine) {
      return true;
    }
    return false;
  }

  public boolean inAllianceZone() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();

    if ((drive.getPose().getX() < redSideLine
            && (allianceColor == AllianceColor.RED || allianceColor == AllianceColor.UNKNOWN))
        || (drive.getPose().getX() > blueSideLine
            && (allianceColor == AllianceColor.BLUE || allianceColor == AllianceColor.UNKNOWN))) {
      return true;
    }
    return false;
  }

  public boolean inOpponentZone() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();
    if ((drive.getPose().getX() < redSideLine && allianceColor == AllianceColor.BLUE)
        || (drive.getPose().getX() > blueSideLine && allianceColor == AllianceColor.RED)) {
      return true;
    }
    return false;
  }
}
