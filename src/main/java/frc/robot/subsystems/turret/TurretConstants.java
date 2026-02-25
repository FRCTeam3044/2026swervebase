package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final int canId = 0;
  public static final int currentLimit = 40;
  public static final int secondaryAbsEncoderDioChannel = 1;

  public static final int turretTeeth = 200;
  public static final int primaryEncoderTeeth = 22;
  public static final int secondaryEncoderTeeth = 27;
  public static final Angle minAngle = Degrees.of(0);
  public static final Angle maxAngle = Degrees.of(360);
  public static final int minPosition = 0;
  public static final int maxPosition = 1; // 1 mernillion
}
