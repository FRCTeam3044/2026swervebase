package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.LEDs.LEDsConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public class LEDsIORio implements LEDsIO {
  private final AddressableLED LEDStrip = new AddressableLED(PWM);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

  private final double blinkSpeed = 0.25;
  private final double spinSpeed = 100;

  public LEDsIORio() {
    LEDStrip.setLength(buffer.getLength());
    LEDStrip.setColorOrder(ColorOrder.kRGB);
    LEDStrip.start();
  }

  @Override
  public void setSolidColor(LEDPattern color) {
    // LEDPattern LEDColor = LEDPattern.solid(color);
    color.applyTo(buffer);
    LEDStrip.setData(buffer);
  }

  @Override
  public void setAlternatingColors(Color color1, Color color2) {
    // every other LED
    LEDPattern pattern = LEDPattern.solid(color1);
    pattern.applyTo(buffer);
    for (int i = 0; i < buffer.getLength(); i++) {
      if (i % 2 == 0) {
        buffer.setLED(i, color1);
      } else {
        buffer.setLED(i, color2);
      }
    }
    LEDStrip.setData(buffer);
  }

  @Override
  public void setSpinningColor(Color color1, Color color2) {
    LEDPattern step = LEDPattern.steps(Map.of(0, color1, 0.83, color2));
    LEDPattern pattern = step.scrollAtRelativeSpeed(Percent.per(Seconds).of(spinSpeed));
    pattern.applyTo(buffer);
    LEDStrip.setData(buffer);
  }

  @Override
  public void setBlinkingColor(Color color) {
    // Create and apply the base blinking pattern
    LEDPattern base = LEDPattern.solid(color);
    LEDPattern pattern = base.blink(Seconds.of(blinkSpeed));
    pattern.applyTo(buffer);

    LEDStrip.setData(buffer);
  }
}
