package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public static final int leaderCanId = 19;
  public static final int followerCanId = 20;
  public static final int currentLimit = 80;
  public static final double maxSpeed = 660; // RPM

  public static final double kS = 0.069052;
  public static final double kV = 0.0018;
  public static final double kA = 0.00023872;

  // since these are used to constrain a profiled PID controller of velocity, the
  // "velocity" constraint is actually acceleration and the "acceleration"
  // constraint is actually jerk
  public static final double maxJerk = 100000;
  public static final double maxAcceleration = 100000;
}
