package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;

public class ClimberIOSpark implements ClimberIO {
  private final SparkMax motor =
      new SparkMax(ClimberConstants.canId, MotorType.kBrushless); // Fill in later

  private final RelativeEncoder climbEncoder = motor.getEncoder();
  // No current angle because it will be 0'd

  // Skipping the HoodIOSpark connecting thing
  public void setSpeed(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void setClimberPos(int height) {}
}
