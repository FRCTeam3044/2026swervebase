package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  // Mirrored
  // public static final Vertex mapBlueVertexToRed(Vertex bluePose) {
  // return new Vertex(
  // DriveConstants.pathfinder.map.fieldx - bluePose.x,
  // DriveConstants.pathfinder.map.fieldy - bluePose.y);
  // }

  // Rotated
  public static final Pose2d mapBluePoseToRed(Pose2d bluePose) {
    return new Pose2d(
        DriveConstants.pathfinder.map.fieldx - bluePose.getX(),
        DriveConstants.pathfinder.map.fieldy - bluePose.getY(),
        new Rotation2d(bluePose.getRotation().getRadians() + Math.PI));
  }

  public static final Rotation2d mapBlueRotToRed(Rotation2d blueRot) {
    return new Rotation2d(blueRot.getRadians() + Math.PI);
  }

  // Rotated
  public static final Vertex mapBlueVertexToRed(Vertex bluePose) {
    return new Vertex(
        DriveConstants.pathfinder.map.fieldx - bluePose.x,
        DriveConstants.pathfinder.map.fieldy - bluePose.y);
  }

  public static final Pose3d mapBluePoseToRed(Pose3d bluePose) {
    return new Pose3d(
        DriveConstants.pathfinder.map.fieldx - bluePose.getX(),
        DriveConstants.pathfinder.map.fieldy - bluePose.getY(),
        bluePose.getZ(),
        new Rotation3d(bluePose.getRotation().getX(), bluePose.getRotation().getY(),
            (bluePose.getRotation().getZ() + Math.PI) % (2 * Math.PI)));
  }

  public static Pose2d getPoseForAlliance(Pose2d bluePose) {
    Pose2d redPose = mapBluePoseToRed(bluePose);
    if (alliance == AllianceColor.BLUE) {
      return bluePose;
    } else if (alliance == AllianceColor.RED) {
      return redPose;
    } else {
      if (robotPose.get().getTranslation().getDistance(bluePose.getTranslation()) < robotPose.get().getTranslation()
          .getDistance(redPose.getTranslation())) {
        return bluePose;
      } else {
        return redPose;
      }
    }
  }

  public static Rotation2d getRotForAlliance(Rotation2d blueRot) {
    Rotation2d redPose = mapBlueRotToRed(blueRot);
    if (alliance == AllianceColor.BLUE) {
      return blueRot;
    } else if (alliance == AllianceColor.RED) {
      return redPose;
    } else {
      return blueRot;
    }
  }

  public static Pose3d getPose3dForAlliance(Pose3d bluePose) {
    Pose3d redPose = mapBluePoseToRed(bluePose);
    if (alliance == AllianceColor.BLUE) {
      return bluePose;
    } else if (alliance == AllianceColor.RED) {
      return redPose;
    } else {
      if (robotPose.get().getTranslation().getDistance(bluePose.getTranslation().toTranslation2d()) < robotPose.get()
          .getTranslation()
          .getDistance(redPose.getTranslation().toTranslation2d())) {
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
      if (robotPose.get().getTranslation().getDistance(bluePose.asPose2d().getTranslation()) < robotPose.get()
          .getTranslation().getDistance(redPose.asPose2d().getTranslation())) {
        return bluePose;
      } else {
        return redPose;
      }
    }
  }
}
