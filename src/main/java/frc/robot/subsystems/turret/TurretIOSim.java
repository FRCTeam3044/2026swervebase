package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Radians;
// import static frc.robot.subsystems.turret.TurretConstants.canId;

// import com.revrobotics.sim.SparkFlexSim;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TurretIOSim implements TurretIO {
    // private SparkFlex motor = new SparkFlex(canId, MotorType.kBrushless);
    // private DCMotor gearBox = DCMotor.getNEO(2);
    // private SparkFlexSim sparkFlexSim = new SparkFlexSim(motor, gearBox);

    // FlywheelSim turretSim = new
    // FlywheelSim(LinearSystemId.createFlywheelSystem(gearBox, 1, 0.02), gearBox);

    // private final TrapezoidProfile.Constraints m_constraints = new
    // TrapezoidProfile.Constraints(2.5, 2.5);

    // private final ProfiledPIDController controller = new
    // ProfiledPIDController(12.527, 1, 0.723, m_constraints, 0.5);

    // @Override
    // public void updateInputs(TurretIOInputsAutoLogged inputs) {
    // inputs.angle = Radians.of(turretSim.);
    // }

    // @Override
    // public void setAngle(Angle angle) {
    // motor.set(MathUtil.clamp(controller.calculate(turretSim.getAngleRads() *
    // (180.0 / Math.PI), angle.in(Degrees)),
    // -1.0, 1.0));
    // }

    // @Override
    // public void setPercent(double percent) {

    // }

    // Dummy impl for testing
    private Angle angle = Degrees.of(0);

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.angle = angle;
    }

    @Override
    public void setAngle(Angle angle) {
        this.angle = angle;
    }
}
