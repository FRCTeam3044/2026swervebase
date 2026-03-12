package frc.robot.statemachine.States.Auto;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.AutoTargetUtil;

public class AutoTrajectories {
    public static ArrayList<Pose2d> getLeftToRightNeutral() {
        ArrayList<Pose2d> path = new ArrayList<>();
        Collections.addAll(path, AutoTargetUtil.getLeftNeutral(), AutoTargetUtil.getRightNeutral());
        return path;
    };

    public static ArrayList<Pose2d> getRightToLeftNeutral() {
        ArrayList<Pose2d> path = new ArrayList<>();
        Collections.addAll(path, AutoTargetUtil.getRightNeutral(), AutoTargetUtil.getLeftNeutral());
        return path;
    };

    public static ArrayList<Pose2d> getCloseLeftToRightNeutral() {
        ArrayList<Pose2d> path = new ArrayList<>();
        Collections.addAll(path, AutoTargetUtil.getCloseLeftNeutral(), AutoTargetUtil.getCloseRightNeutral());
        return path;
    };

    public static ArrayList<Pose2d> getCloseRightToLeftNeutral() {
        ArrayList<Pose2d> path = new ArrayList<>();
        Collections.addAll(path, AutoTargetUtil.getCloseRightNeutral(), AutoTargetUtil.getCloseLeftNeutral());
        return path;
    };

    public static ArrayList<Pose2d> getLeftCurve() {
        ArrayList<Pose2d> path = new ArrayList<>();
        Collections.addAll(path, AutoTargetUtil.getLeftNeutral(), AutoTargetUtil.getTopLeftMiddle(),
                AutoTargetUtil.getBottomLeftMiddle(), AutoTargetUtil.getCloseLeftNeutral());
        return path;
    };

    public static ArrayList<Pose2d> getRightCurve() {
        ArrayList<Pose2d> path = new ArrayList<>();
        Collections.addAll(path, AutoTargetUtil.getRightNeutral(), AutoTargetUtil.getTopRightMiddle(),
                AutoTargetUtil.getBottomRightMiddle(), AutoTargetUtil.getCloseRightNeutral());
        return path;
    };
}