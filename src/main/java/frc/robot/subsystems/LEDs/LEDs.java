package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private final LEDsIO io;

  public LEDs(LEDsIO io) {
    this.io = io;
  }

  public Command setPurple() {
    return Commands.run(() -> io.setSolidColor(LEDPattern.solid(Color.kPurple)), this)
        .withName("Set Purple LEDs");
  }
  // red blue and green solid -- yellow blinking shooting and orange blinking intaking autos --
  public Command setYellow() {
    return Commands.run(() -> io.setSolidColor(LEDPattern.solid(Color.kYellow)), this)
        .withName("Set Yellow LEDs");
  }

  public Command setRed() {
    return Commands.run(() -> io.setSolidColor(LEDPattern.solid(Color.kRed)), this)
        .withName("Set Red LEDs");
  }

  public Command setBlue() {
    return Commands.run(() -> io.setSolidColor(LEDPattern.solid(Color.kBlue)), this)
        .withName("Set Blue LEDs");
  }

  public Command setGreen() {
    return Commands.run(() -> io.setSolidColor(LEDPattern.solid(Color.kGreen)), this)
        .withName("Set Green LEDs");
  }

  public Command defaultPattern() {
    return Commands.run(() -> io.setSpinningColor(Color.kPurple, Color.kYellow), this)
        .withName("Spinning LEDs");
  }

  public Command off() {
    return Commands.run(io::setOff, this).withName("LEDs off");
  }

  public Command setBlinkingPurple() {
    return Commands.run(() -> io.setBlinkingColor(Color.kPurple), this)
        .withName("Blinking Purple LEDs");
  }

  public Command setBlinkingYellow() {
    return Commands.run(() -> io.setBlinkingColor(Color.kYellow), this)
        .withName("Blinking Yellow LEDs");
  }

  public Command setBlinkingOrange() {
    return Commands.run(() -> io.setBlinkingColor(Color.kOrange), this)
        .withName("Blinking Orange LEDs");
  }
}
