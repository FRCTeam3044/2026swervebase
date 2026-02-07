// package frc.robot.subsystems.turret;

// import com.revrobotics.sim.SparkFlexSim;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// public class TurretIOSim implements TurretIO {
// private SparkFlex sparkFlex = new SparkFlex(52, MotorType.kBrushless);
// private DCMotor gearBox = DCMotor.getNEO(2);
// private SparkFlexSim sparkFlexSim = new SparkFlexSim(sparkFlex, gearBox);

// SingleJointedArmSim turretSim =
// new SingleJointedArmSim(gearBox, 5.0, 1.0, 0.3, Math.PI, -Math.PI, true, 0,
// null);

// private final TrapezoidProfile.Constraints m_constraints =
// new TrapezoidProfile.Constraints(2.5, 2.5);

// private final ProfiledPIDController controller =
// new ProfiledPIDController(12.527, 1, 0.723, m_constraints, 0.5);

// @Override
// public void updateInputs(TurretIOInputsAutoLogged inputs) {
// inputs.angle = turretSim.getAngleRads();
// }

// @Override
// public void setAngle(double angle) {}
// }
