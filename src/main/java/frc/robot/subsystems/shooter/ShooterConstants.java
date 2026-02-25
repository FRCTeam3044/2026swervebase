package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public static final int leaderCanId = 0;
  public static final int followerCanId = 1;
  public static final int currentLimit = 40;

  public static final double kS = 0.0;
  public static final double kV = 0.0;

  // since these are used to constrain a profiled PID controller of velocity, the
  // "velocity" constraint is actually acceleration and the "acceleration"
  // constraint is actually jolt
  public static final double maxJolt = 360;
  public static final double maxAcceleration = 360;
}
