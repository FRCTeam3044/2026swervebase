package frc.robot.util;

public class AutoAim {
  private PolynomialRegression speedRegression;
  private PolynomialRegression angleRegression;
  private AimMode mode;

  public enum AimMode {
    Linear,
    Polynomial,
  }

  public double[] sampleDistances = new double[] {1.445, 2.006, 2.313, 2.931, 3.326, 4.437, 5.577};

  public double[] sampleSpeeds = new double[] {7.5, 7.7, 7.7, 7.9, 8.2, 8.5, 9.0};

  public double[] sampleAngles = new double[] {80.0, 77.0, 75.0, 71.0, 69.0, 63.0, 57.0};

  public AutoAim(AimMode mode) {
    this.mode = mode;
    switch (mode) {
      case Linear:
        throw new UnsupportedOperationException("Linear mode not implemented yet");
      case Polynomial:
        speedRegression = new PolynomialRegression(sampleDistances, sampleSpeeds, 4);
        angleRegression = new PolynomialRegression(sampleDistances, sampleAngles, 4);
        break;
    }
  }

  public double calculateAngle(double distanceMeters) {
    switch (mode) {
      case Linear:
        throw new UnsupportedOperationException("Linear mode not implemented yet");
      case Polynomial:
        return angleRegression.predict(distanceMeters);
    }

    return -1;
  }

  public double calculateSpeed(double distanceMeters) {
    switch (mode) {
      case Linear:
        throw new UnsupportedOperationException("Linear mode not implemented yet");
      case Polynomial:
        return speedRegression.predict(distanceMeters);
    }

    return -1;
  }
}
