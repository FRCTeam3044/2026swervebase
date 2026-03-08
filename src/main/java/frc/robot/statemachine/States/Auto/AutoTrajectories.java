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
}
