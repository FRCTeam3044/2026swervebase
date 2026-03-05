package frc.robot.util;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;

public class AutoEnums {
  public AutoEnums() {
  }

  public enum AutoSteps {
    LeftClimb(() -> false),
    RightClimb(() -> false),
    ShootToHub(() -> RobotContainer.getInstance().autoStateTimer.get() > 4),
    ShootToAlliedSide(() -> false),
    IntakeNeutralZone(() -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getLeftNeutral())),
    IntakeAllianceZone(() -> false),
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
