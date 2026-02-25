package frc.robot.subsystems.hood;

// mport com.revrobotics.RelativeEncoder;
// mport com.revrobotics.sim.SparkMaxSim;
// mport com.revrobotics.spark.SparkLowLevel.MotorType;
// mport com.revrobotics.spark.SparkMax;
// mport edu.wpi.first.math.controller.PIDController;
// mport edu.wpi.first.math.system.plant.DCMotor;
// mport edu.wpi.first.wpilibj.RobotController;
// mport edu.wpi.first.wpilibj.simulation.BatterySim;
// mport edu.wpi.first.wpilibj.simulation.RoboRioSim;
// mport edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  // private SparkMax sparkMax = new SparkMax(50, MotorType.kBrushless);
  // private DCMotor gearBox = DCMotor.getNEO(2);
  // private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);
  // private RelativeEncoder encoder = sparkMax.getEncoder();

  // public double currentTarget;

  // public final SingleJointedArmSim m_hoodSim = new SingleJointedArmSim(
  // // Simulated motor
  // gearBox,
  // // Gear ratio
  // 1.0,
  // // Moment of inertia of the hood
  // 0.002,
  // 0.3,
  // // Minimum angle (radians)
  // -Math.PI,
  // // Maximum angle (radians)
  // Math.PI,
  // true,
  // 0);

  // private final PIDController m_controller = new PIDController(1, 0, 0);
  // // ArmFeedforward m_feedforward = new ArmFeedforward(/* kS */ 0, /* kG */ 0,
  // /*
  // // kV */ 1);

  // @Override
  // public void updateInputs(HoodIOInputs inputs) {
  // zm_hoodSim.setInput(sparkMax.getAppliedOutput() *
  // RobotController.getBatteryVoltage());
  // m_hoodSim.update(0.020);
  // RoboRioSim.setVInVoltage(
  // BatterySim.calculateDefaultBatteryLoadedVoltage(m_hoodSim.getCurrentDrawAmps()));
  // sparkMaxSim.iterate(
  // ((m_hoodSim.getVelocityRadPerSec()
  // / (
  // /* drumRadius */ 1.0 * 2.0 * Math.PI * /* motorReduction */ 1.0))
  // * 60.0),
  // RobotController.getBatteryVoltage(),
  // 0.02);

  // m_controller.setSetpoint(currentTarget);
  // double pidOutput = m_controller.calculate(m_hoodSim.getAngleRads());
  // // double feedforwardOutput = m_feedforward.calculate(encoder.getPosition(),
  // // m_controller.getSetpoint());
  // sparkMax.setVoltage(pidOutput);

  // // inputs.setpointRotations = currentTarget;
  // // inputs.setpointRads = encoder.getPosition();
  // // inputs.hoodAngleRads = m_hoodSim.getAngleRads();
  // }

  // @Override
  // public void setPosition(double position) {
  // currentTarget = position;
  // }

  private double position = 0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = position;
  }

  @Override
  public void setPosition(double position) {
    this.position = position;
  }
}
