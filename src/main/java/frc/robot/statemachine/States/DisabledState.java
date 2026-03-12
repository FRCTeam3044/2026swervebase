package frc.robot.statemachine.States;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDs.ColorPair;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AllianceUtil.AllianceColor;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class DisabledState extends State {
  private ConfigurableParameter<Boolean> spinInDisabled = new ConfigurableParameter<Boolean>(false, "Spin in disabled");

  public DisabledState(StateMachineBase stateMachine, LEDs leds, Turret turret, Hood hood, Vision vision) {
    super(stateMachine);

    Supplier<Color> allianceColor = () -> {
      AllianceColor alliance = AllianceUtil.getAlliance();
      if (alliance == AllianceColor.UNKNOWN) {
        return Color.kWhite;
      } else if (alliance == AllianceColor.RED) {
        return Color.kRed;
      } else {
        return Color.kFirstBlue;
      }
    };

    Supplier<ColorPair> color = () -> {
      if (!turret.hasReset()) {
        return new ColorPair(Color.kYellow, Color.kYellow);
      }
      if (turret.inHoodDangerZone() && !hood.calibrated()) {
        return new ColorPair(Color.kFirstRed, Color.kOrange);
      }
      if (vision.hasTarget()) {
        return new ColorPair(allianceColor.get(), Color.kGreen);
      }
      return new ColorPair(allianceColor.get(), allianceColor.get());
    };

    startWhenActive(Commands.runOnce(() -> Robot.loopCount = 0));
    t(spinInDisabled::get).and(() -> Robot.loopCount > 10).whileTrue(leds.defaultPattern());
    t(() -> !spinInDisabled.get()).and(() -> Robot.loopCount > 10).whileTrue(leds.setAlternatingColors(color));
  }
}
