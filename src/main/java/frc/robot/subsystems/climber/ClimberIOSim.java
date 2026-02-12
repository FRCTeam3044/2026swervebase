package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

//Not finished

public class ClimberIOSim implements ClimberIO {
        private SparkMax sparkMax = new SparkMax(ClimberConstants.canId, null); // Fill in Later
        // private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);
        private RelativeEncoder encoder = sparkMax.getEncoder();
}