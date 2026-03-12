package frc.robot.statemachine.States;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
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
        return Color.kYellow;
      } else if (alliance == AllianceColor.RED) {
        return Color.kRed;
      } else {
        return Color.kBlue;
      }
    };

    Supplier<ColorPair> color = () -> {
      if (!turret.hasReset()) {
        return new ColorPair(Color.kHotPink, Color.kHotPink);
      }
      if (turret.inHoodDangerZone() && !hood.calibrated()) {
        return new ColorPair(Color.kOrange, Color.kOrange);
      }
      if (vision.hasTarget()) {
        return new ColorPair(allianceColor.get(), Color.kGreen);
      }
      return new ColorPair(allianceColor.get(), allianceColor.get());
    };

    startWhenActive(leds.setAlternatingColors(color).onlyIf(() -> !spinInDisabled.get()));
    startWhenActive(leds.defaultPattern().onlyIf(spinInDisabled::get));
    t(spinInDisabled::get).whileTrue(leds.defaultPattern()).whileFalse(leds.setAlternatingColors(color));
  }
}
