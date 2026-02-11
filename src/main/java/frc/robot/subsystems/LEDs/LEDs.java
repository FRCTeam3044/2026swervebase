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

  public Command defaultPattern() {
    return Commands.run(() -> io.setSpinningColor(Color.kPurple, Color.kYellow), this)
        .withName("Spinning LEDs");
  }

  public Command off() {
    return Commands.run(io::setOff, this).withName("LEDs off");
  }
}
