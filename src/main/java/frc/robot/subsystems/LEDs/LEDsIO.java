package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface LEDsIO {
  @AutoLog
  public static class LEDsIOInputs {
    static int indexOfStr = 0;
    static int indexOfChar = 0;
    static Time timeOfDot = Milliseconds.of(500);
    static Time timeOfDash = Milliseconds.of(1000);
    static Time currentTime;
  }

  public default void setSolidColor(LEDPattern color) {}
  ;

  public default void setSpinningColor(Color color1, Color color2) {}
  ;

  public default void setOff() {}
  ;

  public default void setBlinkingColor(Color color) {}
  ;
}
