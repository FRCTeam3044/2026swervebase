package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;

public class Climber extends SubsystemBase {
    private final ClimberIO io; // Brings in the Climber IO interface
    private final ConfigurableParameter<Double> speed = new ConfigurableParameter<>(0.0, "Climber speed");

    public Climber(ClimberIO io) { // idk just needed for climberio to be final
        this.io = io;
    }

    public Command setSpeed() { // Command factory for moving climber
        return Commands.run(
                () -> {
                    io.setSpeed(speed.get()); // Input smth into speed
                }, this);
    }
}
