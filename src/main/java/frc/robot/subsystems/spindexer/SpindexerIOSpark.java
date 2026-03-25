package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SpindexerIOSpark implements SpindexerIO {
  private final SparkFlex leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
  private final SparkFlex follower = new SparkFlex(followerCanId, MotorType.kBrushless);

  private RelativeEncoder encoderOne = leader.getEncoder();
  private RelativeEncoder encoderTwo = follower.getEncoder();

  public SpindexerIOSpark() {
    tryUntilOk(
        leader,
        5,
        () -> leader.configure(
            SpindexerConfig.leaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    tryUntilOk(
        follower,
        5,
        () -> follower.configure(
            SpindexerConfig.followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    ifOk(leader, leader::getOutputCurrent, (value) -> inputs.leaderApms = value);
    ifOk(follower, follower::getOutputCurrent, (value) -> inputs.followerApms = value);
    ifOk(leader, leader::getAppliedOutput, (value) -> inputs.leaderOutput = value);
    ifOk(follower, follower::getAppliedOutput, (value) -> inputs.followerOutput = value);
    ifOk(leader, encoderOne::getVelocity, (value) -> inputs.leaderSpeed = value);
    ifOk(follower, encoderTwo::getVelocity, (value) -> inputs.followerSpeed = value);
  }

  @Override
  public void setSpeed(double speed) {
    leader.set(speed);
  }
}
