package frc.robot.util;

import java.util.function.BooleanSupplier;

public class AutoEnums {
  public AutoEnums() {
  }

  public enum AutoSteps {
    // These conditions are the END conditions
    AutoClimb(/* Auto is finished */ () -> false),
    ShootToHub(/* Hopper is empty */() -> false),
    ShootToAlliedSide(/* Hopper is empty */() -> false),
    IntakeNeutralZone(/* Hopper is full */() -> false),
    IntakeAllianceZone(/* Hopper is full */() -> false);

    private final BooleanSupplier condition;

    private AutoSteps(BooleanSupplier condition) {
      this.condition = condition;
    }

    public BooleanSupplier getCondition() {
      return condition;
    }
  }
}
