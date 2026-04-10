// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kUSB1, (byte) odometryFrequency);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    Rotation3d normalRotation = new Rotation3d(navX.getRoll(), navX.getPitch(), 0);
    Vector<N3> baseNormalVector = VecBuilder.fill(0, 0, 1);
    Matrix<N3, N1> rotatedNormal = normalRotation.toMatrix().times(baseNormalVector);
    Vector<N3> rotatedNormalVec = new Vector<N3>(rotatedNormal);

    double dotProduct = rotatedNormalVec.dot(baseNormalVector);
    double magnitude = rotatedNormalVec.normF() * baseNormalVector.normF();
    double angleToNormal = Math.acos(dotProduct / magnitude);

    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(-value))
        .toArray(Rotation2d[]::new);

    inputs.navXPitch = navX.getPitch();
    inputs.navXRoll = navX.getRoll();
    inputs.angleToNormal = angleToNormal;
    // inputs.normalVector = rotatedNormalVec;
    inputs.magnitude = magnitude;

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
