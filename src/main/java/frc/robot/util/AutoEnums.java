package frc.robot.util;

import java.util.function.BooleanSupplier;

public class AutoEnums {
  public enum AutoSteps {
    AutoClimb(/* time left is 10 sec */ () -> false),
    ShootToHub(/* filled hopper */ () -> false),
    ShootToAlliedSide(/* filled hopper */ () -> false),
    IntakeNeutralZone(/* out of fuel */ () -> false),
    IntakeAllianceZone(/* out of fuel */ () -> false);

    private final BooleanSupplier condition;

    private AutoSteps(BooleanSupplier condition) {
      this.condition = condition;
    }

    public BooleanSupplier getCondition() {
      return condition;
    }
  }
}
