package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.StateMachine;

public class AutoEnums {
    public AutoEnums() {
    }

    public enum AutoSteps {
        LeftClimb(() -> false),
        RightClimb(() -> false),
        FirstScore(() -> {
            if (RobotContainer.getInstance().autoStateTimer.get() > StateMachine.firstAutoTime.get()) {
                RobotContainer.getInstance().autoStateTimer.stop();
                RobotContainer.getInstance().autoStateTimer.reset();
                return true;
            }
            return false;
        }),

        SecondScore(() -> {
            if (RobotContainer.getInstance().autoStateTimer.get() > StateMachine.secondAutoTime.get()) {
                RobotContainer.getInstance().autoStateTimer.stop();
                RobotContainer.getInstance().autoStateTimer.reset();
                return true;
            }
            return false;
        }),
        LeftSwoop(() -> frc.robot.statemachine.States.ConsolidatedAuto.NeutralPaths.stateComplete
                && RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getCloseLeftNeutral())),
        RightSwoop(() -> frc.robot.statemachine.States.ConsolidatedAuto.NeutralPaths.stateComplete
                && RobotContainer.getInstance().drive.atPose(AutoTargetUtil
                        .getCloseRightNeutral())),
        IntakeDepot(/* TODO: figure out whether to have end condition or not */ () -> false),
        IntakeOutpost(/* TODO: figure out whether to have end condition or not */ () -> false),
        LeftTransition(() -> RobotContainer.getInstance().drive
                .atRotationTight(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)))
                &&
                RobotContainer.getInstance().drive.atPoseTight(AutoTargetUtil.getSafeLeftNeutral())),
        RightTransition(() -> RobotContainer.getInstance().drive
                .atRotationTight(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))) &&
                RobotContainer.getInstance().drive.atPoseTight(AutoTargetUtil.getSafeRightNeutral())),
        LeftCloseTransition(() -> RobotContainer.getInstance().drive
                .atRotationTight(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)))
                &&
                RobotContainer.getInstance().drive.atPoseTight(AutoTargetUtil.getSafeCloseLeftNeutral())),
        RightCloseTransition(() -> RobotContainer.getInstance().drive
                .atRotationTight(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))) &&
                RobotContainer.getInstance().drive.atPoseTight(AutoTargetUtil.getSafeCloseRightNeutral())),
        LeftInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomLeftMiddle())),
        RightInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomRightMiddle())),
        LeftHub(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getLeftHubPos())),
        RightHub(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getRightHubPos())),
        // FarLeftInOut(() ->
        // RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getTopLeftMiddle())),
        // FarRightInOut(() ->
        // RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getTopRightMiddle())),
        EmptyState(() -> false);

        private final BooleanSupplier condition;

        private AutoSteps(BooleanSupplier condition) {
            this.condition = condition;
        }

        public BooleanSupplier getCondition() {
            return condition;
        }
    }
}