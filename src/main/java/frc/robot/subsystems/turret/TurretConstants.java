package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final int canId = 15;
  public static final int currentLimit = 8;
  public static final int primaryAbsEncoderDioChannel = 0;
  public static final int secondaryAbsEncoderDioChannel = 1;

  public static final Angle primaryAbsEncoderZero = Degrees.of(57.51667);
  public static final Angle secondaryAbsEncoderZero = Degrees.of(339.797861);

  public static final int turretTeeth = 200;
  public static final int primaryEncoderTeeth = 22;
  public static final int secondaryEncoderTeeth = 27;
  public static final Angle minAngle = Degrees.of(39.6);
  public static final Angle maxAngle = Degrees.of(354.6);
  public static final double minPosition = 0.658222;
  public static final double maxPosition = -0.038237; // 1 mernillion

  public static final double kS = 0.0;
  public static final double kV = 0.0;

  public static final double maxVelocity = 360;
  public static final double maxAcceleration = 360;
}
