package frc.robot.subsystems.climber;
import static frc.robot.util.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import me.nabdev.oxconfig.ConfigurableParameter;


public class ClimberIOSpark implements ClimberIO {
    private final SparkMax motor = new SparkMax(ClimberConstants.canId, MotorType.Brushless); //Fill in later

    private final RelativeEncoder climbEncoder = motor.getEncoder();
    private final SparkLimitSwitch bottomLimit = motor.getReverseLimitSwitch();
    private final ConfigurableParameter<Double> runSpeedUp = new ConfigurableParameter<>(0.0, "Climber Motor Running Speed");
    // No current angle because it will be 0'd
    private final ConfigurableParameter<Double> runSpeedDown = new ConfigurableParameter<>(-0.0, "Climber Motor Running Speed");
    public void updateInputs(ClimberIOInputs inputs) {
        if (bottomLimit.isPressed()) {
            inputs.bottomLimitPressed = true;
            climbEncoder.setPosition(0); //Reset encoder to 0 when bottom limit is pressed
        }
        inputs.currentPosition = climbEncoder.getPosition();
        ifOk(motor, climbEncoder::getPosition, (value) -> inputs.currentPosition = value);
    }
    //Skipping the HoodIOSpark connecting thing
    public void setSpeed(double speed) {
        motor.set(MathUtil.clamp(speed, -1, 1)); 
    }

    public void setClimberPos(double wantedHeight){
        double currentPosition = climbEncoder.getPosition(); //Get current position
        if (wantedHeight > currentPosition) {
            motor.set(runSpeedUp.get());
        } else if (currentPosition > wantedHeight) {
            motor.set(runSpeedDown.get());
        } else {
            motor.set(0.0); // sets motor to 0
        }
    }
}
