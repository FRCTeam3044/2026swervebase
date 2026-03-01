package frc.robot.util;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;

public class AutoEnums {
  public AutoEnums() {
  }

  public enum AutoSteps {
    // These conditions are the END conditions
    LeftClimb(() -> false),
    RightClimb(() -> false),
    ShootToHub(/* Hopper is empty */ () -> RobotContainer.getInstance().autoStateTimer.get() > 2),
    ShootToAlliedSide(/* Hopper is empty */ () -> false),
    IntakeNeutralZone(
        /* Hopper is full */ () -> RobotContainer.getInstance().drive.atPose(AutoTargetUtil.getRightNeutral())),
    IntakeAllianceZone(/* Hopper is full */ () -> false),
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
