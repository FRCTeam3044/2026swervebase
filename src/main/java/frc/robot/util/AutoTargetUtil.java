package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceUtil.AllianceColor;
import lombok.experimental.ExtensionMethod;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vector;
import me.nabdev.pathfinding.structures.Vertex;

@ExtensionMethod({ GeomUtil.class })
public class AutoTargetUtil {
  public static AutoTargetUtil instance;
  private final Drive drive;

  public final static ConfigurableParameter<Double> blueSideLine = new ConfigurableParameter<Double>(4.0,
      "Blue Side az line");

  public final static ConfigurableParameter<Double> blueBumpSideLine = new ConfigurableParameter<Double>(3.5,
      "Blue bump side az line");

  public final static ConfigurableParameter<Double> blueFastSpeedLine = new ConfigurableParameter<Double>(3.5,
      "Blue fast speed line");

  private Pose3d hub = new Pose3d(new Translation3d(4.62, 8.069 / 2.0, 2), new Rotation3d());
  private Pose3d outpostAllianceTarget = new Pose3d(new Translation3d(1, 1.7, 0), new Rotation3d());
  private Pose3d depotAllianceTarget = new Pose3d(new Translation3d(1, 6.3, 0), new Rotation3d());

  private static Pose2d testNeutralZonePosition = new Pose2d(7, 4, Rotation2d.fromDegrees(0));

  private static POIData leftTower = POIData.createFromRed(1.06223613, 3.20841229, 1.06223613, 2.60841229);
  private static POIData rightTower = POIData.createFromRed(1.06223613, 4.28304359, 1.06223613, 4.88304359);

  private static Pose2d rightNeutral = new Pose2d(8.5, 1.25, Rotation2d.fromDegrees(0));
  private static Pose2d closeRightNeutral = new Pose2d(7.5, 1.25, Rotation2d.fromDegrees(0));
  private static Pose2d safeRightNeutral = new Pose2d(7.75, 0.6, Rotation2d.fromDegrees(0));
  private static Pose2d safeCloseRightNeutral = new Pose2d(6, 0.6, Rotation2d.fromDegrees(0));
  private static Pose2d topRightMiddle = new Pose2d(8.5, 3.2, Rotation2d.fromDegrees(0));
  private static Pose2d bottomRightMiddle = new Pose2d(7.5, 3.2, Rotation2d.fromDegrees(0));
  private static Pose2d rightBehindHub = new Pose2d(7, 3.5, Rotation2d.fromDegrees(0));
  private static Pose2d rightWideHub = new Pose2d(7, 2, Rotation2d.fromDegrees(0));
  private static Pose2d rightAzPoint = new Pose2d(2.5, 0.1, Rotation2d.fromDegrees(0));

  private static Pose2d testBumpPos = new Pose2d(6.8, 2.5, Rotation2d.fromDegrees(0));

  private static POIData depot = POIData.createFromRed(0.4, 5.96503125, 1.06827289, 5.96503125);

  private static Pose2d outpost = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180));

  private static Obstacle redAllianceZone = Obstacle.createObstacle(
      new Vertex(
          DriveConstants.pathfinder.map.fieldx - 3.8, 0),
      new Vertex(DriveConstants.pathfinder.map.fieldx
          - 3.8, 8.0692625),
      new Vertex(16.5410515, 8.0692625),
      new Vertex(16.5410515, 0));

  private static Obstacle blueAllianceZone = Obstacle.createObstacle(
      new Vertex(0, 0),
      new Vertex(0, 8.0692625),
      new Vertex(3.8, 8.0692625),
      new Vertex(3.8, 0));

  private static Obstacle redAllianceZoneSecond = Obstacle.createObstacle(
      new Vertex(13.1, 0),
      new Vertex(13.1, 8.0692625),
      new Vertex(16.5410515, 8.0692625),
      new Vertex(16.5410515, 0));

  private static Obstacle blueAllianceZoneSecond = Obstacle.createObstacle(
      new Vertex(0, 0),
      new Vertex(0, 8.0692625),
      new Vertex(3.4, 8.0692625),
      new Vertex(3.4, 0));

  public AutoTargetUtil(Drive drive) {
    if (instance == null) {
      instance = this;
    }
    this.drive = drive;
  }

  public static Pose2d mirrorY(Pose2d rightSide) {
    return new Pose2d(rightSide.getX(), DriveConstants.pathfinder.map.fieldy - rightSide.getY(),
        rightSide.getRotation());
  }

  public Pose3d getHub() {
    return AllianceUtil.getPose3dForAlliance(hub);
  }

  public static Pose2d getLeftNeutral() {
    return AllianceUtil.getPoseForAlliance(mirrorY(rightNeutral));
  }

  public static Pose2d getRightNeutral() {
    return AllianceUtil.getPoseForAlliance(rightNeutral);
  }

  public static Pose2d getCloseLeftNeutral() {
    return AllianceUtil.getPoseForAlliance(mirrorY(closeRightNeutral));
  }

  public static Pose2d getCloseRightNeutral() {
    return AllianceUtil.getPoseForAlliance(closeRightNeutral);
  }

  public static Pose2d getSafeLeftNeutral() {
    return AllianceUtil.getPoseForAlliance(mirrorY(safeRightNeutral));
  }

  public static Pose2d getSafeRightNeutral() {
    return AllianceUtil.getPoseForAlliance(safeRightNeutral);
  }

  public static Pose2d getSafeCloseLeftNeutral() {
    return AllianceUtil.getPoseForAlliance(mirrorY(safeCloseRightNeutral));
  }

  public static Pose2d getSafeCloseRightNeutral() {
    return AllianceUtil.getPoseForAlliance(safeCloseRightNeutral);
  }

  public static Pose2d getTopLeftMiddle() {
    return AllianceUtil.getPoseForAlliance(mirrorY(topRightMiddle));
  }

  public static Pose2d getTopRightMiddle() {
    return AllianceUtil.getPoseForAlliance(topRightMiddle);
  }

  public static Pose2d getBottomLeftMiddle() {
    return AllianceUtil.getPoseForAlliance(mirrorY(bottomRightMiddle));
  }

  public static Pose2d getBottomRightMiddle() {
    return AllianceUtil.getPoseForAlliance(bottomRightMiddle);
  }

  public static Pose2d getLeftHubPos() {
    return AllianceUtil.getPoseForAlliance(mirrorY(rightBehindHub));
  }

  public static Pose2d getRightHubPos() {
    return AllianceUtil.getPoseForAlliance(rightBehindHub);
  }

  public static Pose2d getWideLeftHub() {
    return AllianceUtil.getPoseForAlliance(mirrorY(rightWideHub));
  }

  public static Pose2d getWideRightHub() {
    return AllianceUtil.getPoseForAlliance(rightWideHub);
  }

  public static Pose2d getLeftBumpPos() {
    return AllianceUtil.getPoseForAlliance(mirrorY(testBumpPos));
  }

  public static Pose2d getRightBumpPos() {
    return AllianceUtil.getPoseForAlliance(testBumpPos);
  }

  public static Pose2d getLeftAzPoint() {
    return AllianceUtil.getPoseForAlliance(mirrorY(rightAzPoint));
  }

  public static Pose2d getRightAzPoint() {
    return AllianceUtil.getPoseForAlliance(rightAzPoint);
  }

  public static Pose2d getNeutralZone() {
    return AllianceUtil.getPoseForAlliance(testNeutralZonePosition);
  }

  public static POIData getLeftTower() {
    return leftTower;
  }

  public static POIData getRightTower() {
    return rightTower;
  }

  public static POIData getDepot() {
    return depot;
  }

  public static Pose2d getOutpost() {
    return AllianceUtil.getPoseForAlliance(outpost);
  }

  public static Obstacle allianceSide() {
    if (AllianceUtil.getAlliance() == AllianceColor.BLUE) {
      return blueAllianceZone;
    } else {
      return redAllianceZone;
    }
  }

  public static Obstacle allianceSideSecondShoot() {
    if (AllianceUtil.getAlliance() == AllianceColor.BLUE) {
      return blueAllianceZoneSecond;
    } else {
      return redAllianceZoneSecond;
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

  public Pose2d getTurretPose() {
    return drive.getPose()
        .transformBy(ShotCalculator.robotToTurret.toTransform2d());
  }

  public static double redSideLine() {
    return DriveConstants.pathfinder.map.fieldx - blueSideLine.get();
  }

  public static double blueSideLine() {
    return blueSideLine.get();
  }

  public static double redBumpSideLine() {
    return DriveConstants.pathfinder.map.fieldx - blueBumpSideLine.get();
  }

  public static double blueBumpSideLine() {
    return blueBumpSideLine.get();
  }

  public static double redFastSpeedLine() {
    return DriveConstants.pathfinder.map.fieldx - blueFastSpeedLine.get();
  }

  public static double blueFastSpeedLine() {
    return blueFastSpeedLine.get();
  }

  public boolean pastFastLine() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();

    return (this.getTurretPose().getX() > redFastSpeedLine()
        && (allianceColor == AllianceColor.RED || allianceColor == AllianceColor.UNKNOWN))
        || (this.getTurretPose().getX() < blueFastSpeedLine()
            && (allianceColor == AllianceColor.BLUE || allianceColor == AllianceColor.UNKNOWN));
  }

  public boolean pastBump() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();

    return (this.getTurretPose().getX() > redBumpSideLine()
        && (allianceColor == AllianceColor.RED || allianceColor == AllianceColor.UNKNOWN))
        || (this.getTurretPose().getX() < blueBumpSideLine()
            && (allianceColor == AllianceColor.BLUE || allianceColor == AllianceColor.UNKNOWN));
  }

  public boolean inNeutralZone() {
    if (this.getTurretPose().getX() < redSideLine() && this.getTurretPose().getX() > blueSideLine()) {
      return true;
    }
    return false;
  }

  public boolean inAllianceZone() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();

    if ((this.getTurretPose().getX() > redSideLine()
        && (allianceColor == AllianceColor.RED || allianceColor == AllianceColor.UNKNOWN))
        || (this.getTurretPose().getX() < blueSideLine()
            && (allianceColor == AllianceColor.BLUE || allianceColor == AllianceColor.UNKNOWN))) {
      return true;
    }
    return false;
  }

  public boolean inOpponentZone() {
    AllianceColor allianceColor = AllianceUtil.getAlliance();
    if ((this.getTurretPose().getX() > redSideLine() && allianceColor == AllianceColor.BLUE)
        || (this.getTurretPose().getX() < blueSideLine() && allianceColor == AllianceColor.RED)) {
      return true;
    }
    return false;
  }

  public record POIData(Vertex pos, Vector normal) {
    public static POIData create(Vertex pos1, Vertex pos2) {
      return new POIData(pos1, pos2.createVectorFrom(pos1).normalize());
    }

    public static POIData create(double pos1x, double pos1y, double pos2x, double pos2y) {
      return create(new Vertex(pos1x, pos1y), new Vertex(pos2x, pos2y));
    }

    public static POIData createFromRed(double pos1x, double pos1y, double pos2x, double pos2y) {
      return create(new Vertex(pos1x, pos1y), new Vertex(pos2x, pos2y));
    }

    public Vector perpindicular(boolean flipped) {
      if (flipped) {
        return new Vector(normal().y, -normal().x).normalize();
      }
      return new Vector(-normal().y, normal().x).normalize();
    }

    public Pose2d poseWithRot(double distance, Rotation2d rotation) {
      Vertex robotPos = pos().moveByVector(normal().scale(distance));
      return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
    }

    public Pose2d poseFacing(double distance, boolean flipped) {
      Vertex robotPos = pos().moveByVector(normal().scale(distance));
      double sign = flipped ? 1 : -1;
      Rotation2d rotation = Rotation2d.fromRadians(Math.atan2(sign * normal().y, sign * normal().x));
      return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
    }

    public Vertex vertexFacing(double distance) {
      Vertex robotPos = pos().moveByVector(normal().scale(distance));
      return AllianceUtil.getVertexForAlliance(robotPos);
    }

    public Pose2d offsetPoseFacing(double distance, boolean flipped, double offset, boolean perpFlipped) {
      Vertex robotPos = pos().moveByVector(normal().scale(distance))
          .moveByVector(perpindicular(perpFlipped).scale(offset));
      double sign = flipped ? 1 : -1;
      Rotation2d rotation = Rotation2d.fromRadians(Math.atan2(sign * normal().y, sign * normal().x));
      return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
    }
  }
}
