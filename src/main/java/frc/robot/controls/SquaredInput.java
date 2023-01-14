package frc.robot.controls;

public class SquaredInput implements InputScaler {
  private final double deadbandLimit;

  public SquaredInput(double deadbandLimit) {
    this.deadbandLimit = deadbandLimit;
  }

  public double scale(double input) {
    double numeratorFactor = (Math.abs(input) - deadbandLimit);
    double denominatorFactor = 1 - deadbandLimit;
    double sign = (input >= 0 ? 1 : -1);

    return sign * ((numeratorFactor * numeratorFactor) / (denominatorFactor * denominatorFactor));
  }
}
