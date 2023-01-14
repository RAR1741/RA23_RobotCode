package frc.robot.controls.controllers;

public class DriverController extends FilteredController {
  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  private final double kTriggerActivationThreshold = 0.5;

  // Drive
  public double getForwardAxis() {
    return this.getFilteredAxis(1);
  }

  public double getStrafeAxis() {
    return this.getFilteredAxis(2);
  }

  public double getTurnAxis() {
    return this.getFilteredAxis(4);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > kTriggerActivationThreshold;
  }

  // Intake
  public boolean getWantsIntakeOpen() {
    return this.getLeftBumper();
  }

  public boolean getWantsIntakeClose() {
    return this.getRightBumper();
  }
}
