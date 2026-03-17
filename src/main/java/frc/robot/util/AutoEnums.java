package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AutoEnums {
    private static ConfigurableParameter<Double> firstAutoTime = new ConfigurableParameter<>(2.0,
            "First auto shot time");

    private static ConfigurableParameter<Double> secondAutoTime = new ConfigurableParameter<>(4.0,
            "Second auto shot time");

    public AutoEnums() {
    }

    public enum AutoSteps {
        LeftClimb(() -> false),
        RightClimb(() -> false),
        FirstScore(() -> {
            if (RobotContainer.getInstance().autoStateTimer.get() > firstAutoTime.get()) {
                RobotContainer.getInstance().autoStateTimer.stop();
                RobotContainer.getInstance().autoStateTimer.reset();
                return true;
            }
            return false;
        }),

        SecondScore(() -> {
            if (RobotContainer.getInstance().autoStateTimer.get() > secondAutoTime.get()) {
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
                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)))
                &&
                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeLeftNeutral())),
        RightTransition(() -> RobotContainer.getInstance().drive
                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))) &&
                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeRightNeutral())),
        // LeftFirstTransition(() -> RobotContainer.getInstance().drive
        // .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180)))
        // &&
        // RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeLeftNeutral())),
        // RightFirstTransition(() -> RobotContainer.getInstance().drive
        // .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180))) &&
        // RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeRightNeutral())),
        LeftInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomLeftMiddle())),
        RightInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomRightMiddle())),
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