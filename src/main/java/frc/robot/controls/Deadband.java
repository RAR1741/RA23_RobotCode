package frc.robot.controls;

public class Deadband implements InputScaler {
  private final double deadbandLimit;

  public Deadband(double deadbandLimit) {
    this.deadbandLimit = deadbandLimit;
  }

  public double scale(double input) {
    return Math.abs(input) > deadbandLimit ? input : 0;
  }
}
