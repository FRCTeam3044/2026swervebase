package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerIOSpark implements SpindexerIO {
  private final SparkFlex bottom = new SparkFlex(leaderCanId, MotorType.kBrushless);
  private final SparkFlex top = new SparkFlex(followerCanId, MotorType.kBrushless);

  private RelativeEncoder encoderOne = bottom.getEncoder();
  private RelativeEncoder encoderTwo = top.getEncoder();

  public SpindexerIOSpark() {
    tryUntilOk(
        bottom,
        5,
        () -> bottom.configure(
            SpindexerConfig.leaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    tryUntilOk(
        top,
        5,
        () -> top.configure(
            SpindexerConfig.followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    ifOk(bottom, bottom::getOutputCurrent, (value) -> inputs.bottomApms = value);
    ifOk(top, top::getOutputCurrent, (value) -> inputs.topApms = value);
    ifOk(bottom, bottom::getAppliedOutput, (value) -> inputs.bottomOutput = value);
    ifOk(top, top::getAppliedOutput, (value) -> inputs.topOutput = value);
    ifOk(bottom, encoderOne::getVelocity, (value) -> inputs.bottomSpeed = value);
    ifOk(top, encoderTwo::getVelocity, (value) -> inputs.topSpeed = value);
  }

  @Override
  public void setTopSpeed(double speed) {
    top.set(speed);
  }

  @Override
  public void setBottomSpeed(double speed) {
    bottom.set(speed);
  }

}
