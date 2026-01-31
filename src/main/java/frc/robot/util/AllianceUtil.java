package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;
import me.nabdev.pathfinding.structures.Vertex;

public class AllianceUtil {
  public static enum AllianceColor {
    UNKNOWN,
    BLUE,
    RED
  }

  private static AllianceColor alliance;
  private static Supplier<Pose2d> robotPose;

  public static void setRobot(Supplier<Pose2d> robotPose) {
    AllianceUtil.robotPose = robotPose;
  }

  public static void setAlliance() {
    if (DriverStation.getAlliance().isEmpty()) {
      alliance = AllianceColor.UNKNOWN;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      alliance = AllianceColor.BLUE;
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      alliance = AllianceColor.RED;
    } else {
      alliance = AllianceColor.UNKNOWN;
    }
  }

  public static void setIfUnknown() {
    if (alliance == AllianceColor.UNKNOWN) {
      setAlliance();
    }
  }

  public static AllianceColor getAlliance() {
    if (alliance == null) {
      return AllianceColor.UNKNOWN;
    }
    return alliance;
  }

  // Mirrored
  // private static final Pose2d mapBluePoseToRed(Pose2d bluePose) {
  // return new Pose2d(
  // DriveConstants.pathfinder.map.fieldx - bluePose.getX(),
  // bluePose.getY(),
  // new Rotation2d(-(bluePose.getRotation().getRadians() - (Math.PI / 2)) +
  // (Math.PI / 2)));
  // }

  // Rotated
  public static final Pose2d mapBluePoseToRed(Pose2d bluePose) {
    return new Pose2d(
        DriveConstants.pathfinder.map.fieldx - bluePose.getX(),
        DriveConstants.pathfinder.map.fieldy - bluePose.getY(),
        new Rotation2d((bluePose.getRotation().getRadians() + Math.PI) % (2 * Math.PI)));
  }

  public static final Vertex mapBlueVertexToRed(Vertex bluePose) {
    return new Vertex(
        DriveConstants.pathfinder.map.fieldx - bluePose.x,
        DriveConstants.pathfinder.map.fieldy - bluePose.y);
  }

  public static Pose2d getPoseForAlliance(Pose2d bluePose) {
    Pose2d redPose = mapBluePoseToRed(bluePose);
    if (alliance == AllianceColor.BLUE) {
      return bluePose;
    } else if (alliance == AllianceColor.RED) {
      return redPose;
    } else {
      if (robotPose.get().getTranslation().getDistance(bluePose.getTranslation())
          < robotPose.get().getTranslation().getDistance(redPose.getTranslation())) {
        return bluePose;
      } else {
        return redPose;
      }
    }
  }

  public static Vertex getVertexForAlliance(Vertex bluePose) {
    Vertex redPose = mapBlueVertexToRed(bluePose);
    if (alliance == AllianceColor.BLUE) {
      return bluePose;
    } else if (alliance == AllianceColor.RED) {
      return redPose;
    } else {
      if (robotPose.get().getTranslation().getDistance(bluePose.asPose2d().getTranslation())
          < robotPose.get().getTranslation().getDistance(redPose.asPose2d().getTranslation())) {
        return bluePose;
      } else {
        return redPose;
      }
    }
  }
}
