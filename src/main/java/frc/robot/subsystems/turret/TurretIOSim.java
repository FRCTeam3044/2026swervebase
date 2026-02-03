package frc.robot.subsystems.turret;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {
    private SparkMax sparkMax = new SparkMax(52, MotorType.kBrushless);
    private DCMotor gearBox = DCMotor.getNEO(2);
    private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);

    SingleJointedArmSim turretSim = new SingleJointedArmSim(gearBox, 5.0, 1.0,
            0.3, Math.PI, -Math.PI, true, 0, null);

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(2.5,
            2.5);

    private final ProfiledPIDController controller = new ProfiledPIDController(12.527, 1, 0.723, m_constraints, 0.5);

    @Override
    public void updateInputs(TurretIOInputsAutoLogged inputs) {
        inputs.angle = turretSim.getAngleRads();
    }

    @Override
    public void setAngle(double angle) {
    }
}
