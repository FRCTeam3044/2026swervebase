package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.StateMachine;

public class AutoEnums {
        public AutoEnums() {
        }

        public enum AutoSteps {
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
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)))
                                &&
                                RobotContainer.getInstance().drive.atPoseTight(AutoTargetUtil.getSafeLeftNeutral())),
                RightTransition(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))) &&
                                RobotContainer.getInstance().drive.atPoseTight(AutoTargetUtil.getSafeRightNeutral())),
                LeftAzPoint(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180)))
                                &&
                                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getLeftAzPoint())),
                RightAzPoint(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180)))
                                && RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getRightAzPoint())),
                LeftCloseTransition(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)))
                                &&
                                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeCloseLeftNeutral())),
                RightCloseTransition(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))) &&
                                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeCloseRightNeutral())),
                LeftReverseTransition(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180)))
                                &&
                                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeCloseLeftNeutral())),
                RightReverseTransition(() -> RobotContainer.getInstance().drive
                                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180))) &&
                                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeCloseRightNeutral())),
                LeftInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomLeftMiddle())),
                RightInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomRightMiddle())),
                FarLeftInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getTopLeftMiddle())),
                FarRightInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getTopRightMiddle())),
                LeftHub(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getLeftHubPos())),
                RightHub(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getRightHubPos())),
                LeftToRight(() -> false),
                RightToLeft(() -> false),
                LeftHubSteal(() -> false),
                RightHubSteal(() -> false),
                RightBumpTransition(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getRightBumpPos())
                                && RobotContainer.getInstance().drive
                                                .atRotation(AllianceUtil
                                                                .getRotForAlliance(Rotation2d.fromDegrees(180)))),
                LeftBumpTransition(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getLeftBumpPos())
                                && RobotContainer.getInstance().drive
                                                .atRotation(AllianceUtil
                                                                .getRotForAlliance(Rotation2d.fromDegrees(180)))),
                // RobotContainer.getInstance().autoTargetUtil.pastBump() &&
                OverBump(() -> {
                        if (RobotContainer.getInstance().drive.pastBump) {
                                RobotContainer.getInstance().drive.pastBump = false;
                                RobotContainer.getInstance().drive.bumpTimer.stop();
                                RobotContainer.getInstance().drive.bumpTimer.reset();
                                return true;
                        }
                        return false;
                }),
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