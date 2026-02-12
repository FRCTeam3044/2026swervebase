package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOSim implements ShooterIO {
    private AngularVelocity velocity = RPM.of(0.0);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leaderVelocity = velocity;
        inputs.followerVelocity = velocity;
    }

    @Override
    public void setSpeed(AngularVelocity speed) {
        this.velocity = speed;
    }
}
