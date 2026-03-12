package frc.robot.subsystems.LEDs;

import static frc.robot.subsystems.LEDs.LEDsConstants.length;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AllianceUtil.AllianceColor;

public class LEDs extends SubsystemBase {

  private final LEDsIO io;

  public LEDs(LEDsIO io) {
    this.io = io;
  }

  public Command defaultPattern() {
    return Commands.run(() -> io.setSpinningColor(Color.kPurple, Color.kYellow), this)
        .withName("Spinning LEDs").ignoringDisable(true);
  }

  public Command setSolidColor(Supplier<Color> color) {
    return Commands.run(() -> io.setSolidColor(LEDPattern.solid(color.get())), this)
        .withName("Set Solid Color").ignoringDisable(true);
  }

  public Command setBlinkingColor(Color color) {
    return Commands.run(() -> io.setBlinkingColor(color), this)
        .withName("Blinking Purple LEDs");
  }

  public record ColorPair(Color color1, Color color2) {
  }

  public Command setAlternatingColors(Supplier<ColorPair> colorPair) {
    return Commands
        .run(() -> io.setAlternatingColors(colorPair.get().color1(), colorPair.get().color2()),
            this)
        .withName("Set alternating colors").ignoringDisable(true);
  }
}
