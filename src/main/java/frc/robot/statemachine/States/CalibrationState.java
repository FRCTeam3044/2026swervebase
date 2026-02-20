package frc.robot.statemachine.States;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import me.nabdev.oxconfig.ConfigurableClass;
import me.nabdev.oxconfig.ConfigurableClassParam;
import me.nabdev.oxconfig.OxConfig;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

public class CalibrationState extends State implements ConfigurableClass {
    private ConfigurableClassParam<Double> calibrationShotFlywheelSpeed = new ConfigurableClassParam<>(this, 3000.0,
            "CalibrationShotFlywheelSpeed");
    private ConfigurableClassParam<Double> calibrationShotHoodPosition = new ConfigurableClassParam<>(this, 0.5,
            "CalibrationShotHoodPosition");
    private ConfigurableClassParam<Double> calibrationShotTurretAngle = new ConfigurableClassParam<>(this, 0.0,
            "CalibrationShotTurretAngle");

    private final List<ConfigurableClassParam<?>> parameters = new ArrayList<>(
            Arrays.asList(calibrationShotFlywheelSpeed, calibrationShotHoodPosition,
                    calibrationShotTurretAngle));

    public CalibrationState(StateMachineBase stateMachine, CommandXboxController rawController, Shooter shooter,
            Turret turret, Hood hood, Kicker kicker, Spindexer spindexer) {
        super(stateMachine);

        SmartXboxController controller = new SmartXboxController(rawController, loop);
        controller.leftTrigger()
                .whileTrue(Commands.parallel(shooter.runSpeed(() -> RPM.of(calibrationShotFlywheelSpeed.get())),
                        turret.setAngle(() -> Degrees.of(calibrationShotTurretAngle.get())),
                        hood.setPosition(() -> calibrationShotHoodPosition.get())).withName("Calibration aiming"));
        controller.rightTrigger().whileTrue(kicker.shootKicker());
        controller.rightTrigger().whileFalse(kicker.blockKicker());
        startWhenActive(kicker.blockKicker().onlyWhile(controller.rightTrigger().negate()).withName("Block Kicker"));
        startWhenActive(spindexer.setSpeed());

        controller.a().onTrue(Commands.runOnce(() -> {
            var key = "Test Parameter " + (parameters.size() - 2);
            var param = new ConfigurableClassParam<>(this, 0,
                    key);
            parameters.add(param);
            OxConfig.registerClassParameter(getKey() + "/" + key, param);
        }).withName("Record Shot"));

        OxConfig.registerConfigurableClass(this);
    }

    @Override
    public List<ConfigurableClassParam<?>> getParameters() {
        return parameters;
    }

    @Override
    public String getKey() {
        return "CalibrationState";
    }
}
