package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final int canId = 15;
  public static final int currentLimit = 60;
  public static final int primaryAbsEncoderDioChannel = 0;
  public static final int secondaryAbsEncoderDioChannel = 1;

  public static final Angle primaryAbsEncoderZero = Degrees.of(57.51667);
  public static final Angle secondaryAbsEncoderZero = Degrees.of(339.797861);

  public static final int turretTeeth = 200;
  public static final int primaryEncoderTeeth = 22;
  public static final int secondaryEncoderTeeth = 27;
  // public static final Angle minAngle = Degrees.of(64.8);
  // public static final Angle maxAngle = Degrees.of(324);
  public static final Angle minAngle = Degrees.of(74.8);
  public static final Angle maxAngle = Degrees.of(314);
  public static final double minPosition = 3.6653;
  public static final double maxPosition = 36.3256; // 1 mernillion

  public static final double kS = 0.0;
  public static final double kV = 0.18;

  public static final double maxVelocity = 30;
  public static final double maxAcceleration = 60;
}
