// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

import com.studica.frc.Navx;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final Navx navX = new Navx(Navx.Port.kUSB1);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> navX.getAngle().in(Degrees));

    navX.enableOptionalMessages(true,
        true,
        true,
        true,
        false,
        true,
        true,
        true,
        true,
        true);
    navX.setODRHz(500);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true; // NavX library doesn't have a way to check
    inputs.temperature = navX.getTemperature();
    inputs.yawPosition = new Rotation2d(navX.getAngle().unaryMinus());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getAngularVel()[1].in(RadiansPerSecond));

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(-value))
        .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
