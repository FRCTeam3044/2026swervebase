package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.statemachine.States.Auto.LeftFirstTransition;
import frc.robot.statemachine.States.Auto.RightInOut;
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
        ShootToHub(() -> {
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
        ShootToAlliedSide(() -> false),
        LeftIntake(() -> frc.robot.statemachine.States.Auto.IntakeNeutralLeft.stateComplete
                && RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getCloseLeftNeutral())),
        RightIntake(() -> frc.robot.statemachine.States.Auto.IntakeNeutralRight.stateComplete
                && RobotContainer.getInstance().drive.atPose(AutoTargetUtil
                        .getCloseRightNeutral())),
        IntakeDepot(() -> false),
        IntakeOutpost(() -> false),
        IntakeAllianceZone(() -> false),
        LeftTransition(() -> RobotContainer.getInstance().drive
                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0)))
                &&
                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeLeftNeutral())),
        RightTransition(() -> false
        // RobotContainer.getInstance().drive
        // .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(0))) &&
        // RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeRightNeutral())
        ),
        LeftFirstTransition(() -> RobotContainer.getInstance().drive
                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180)))
                &&
                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeLeftNeutral())),
        RightFirstTransition(() -> RobotContainer.getInstance().drive
                .atRotation(AllianceUtil.getRotForAlliance(Rotation2d.fromDegrees(180))) &&
                RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getSafeRightNeutral())),
        LeftInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomLeftMiddle())),
        RightInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getBottomRightMiddle())),
        FarLeftInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getTopLeftMiddle())),
        FarRightInOut(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getTopRightMiddle())),
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