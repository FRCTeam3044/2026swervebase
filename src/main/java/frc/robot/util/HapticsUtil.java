package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class HapticsUtil {
  private HapticsUtil() {
  }

  public static Command rumblePulses(CommandXboxController controller, int pulses, double onTime, double offTime) {
    List<Command> cmds = new ArrayList<>();
    for (int i = 0; i < pulses; i++) {
      cmds.add(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1)));
      cmds.add(Commands.waitSeconds(onTime));
      cmds.add(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
      if (i < pulses - 1) {
        cmds.add(Commands.waitSeconds(offTime));
      }
    }
    return Commands.sequence(cmds.toArray(new Command[0]));
  }

  public static Command rumbleForTime(CommandXboxController controller, double time) {
    return Commands.sequence(
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1)),
        Commands.waitSeconds(time),
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
  }
}
