package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

public class HoodIOSpark implements HoodIO { // Outlines the HoodIOSpark class
  private final SparkMax motor = new SparkMax(HoodConstants.canId, MotorType.kBrushless); // Creates a new sparkmax motor and puts it into "motor"

  private final ConfigurableProfiledPIDController hoodController = // Creates a new PID controller type
      new ConfigurableProfiledPIDController(
          0.0, 0.1, 0.0, new Constraints(0, 0), "Hood Controller");

  private final RelativeEncoder hoodEncoder = motor.getEncoder(); //Creates RelativeEncoder and sets it to "hoodEncoder"
  private double currentAngleDeg; // justs creates the currentAngleDeg and sets the type to double

  public HoodIOSpark() { // Attempts to connect with the Hood 
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                HoodConfig.hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(HoodIOInputs inputs) { // method execution of updateInputs
    currentAngleDeg = Math.toDegrees(hoodEncoder.getPosition()); // Sets current angle var to be what its at using math
    inputs.angleDeg = currentAngleDeg; // idk puts currentAngleDeg into inputs.angleDeg 
  }

  @Override
  public void setAngle(double angle) { // Command execution of setAngle()
    motor.set(MathUtil.clamp(hoodController.calculate(currentAngleDeg, angle), -1, 1)); // Sets the motor to the new pos based on current pos
  }
}
