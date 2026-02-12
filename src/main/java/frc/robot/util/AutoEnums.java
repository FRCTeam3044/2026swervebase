package frc.robot.util;

import java.util.function.BooleanSupplier;

public class AutoEnums {
  public AutoEnums() {
  }

  public enum AutoSteps {
    AutoClimb(() -> false),
    ShootToHub(() -> false),
    ShootToAlliedSide(() -> false),
    IntakeNeutralZone(() -> false),
    IntakeAllianceZone(() -> false);

    private final BooleanSupplier condition;

    private AutoSteps(BooleanSupplier condition) {
      this.condition = condition;
    }

    public BooleanSupplier getCondition() {
      return condition;
    }
  }
}
