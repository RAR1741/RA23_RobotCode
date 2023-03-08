package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsDefaultState() {
    return this.getRawButton(2);
  }

  public double getArmHorizontalChange(double strength) {
    return this.getFilteredAxis(0) * strength;
  }

  public double getArmVerticalChange(double strength) {
    return this.getFilteredAxis(5) * -strength;
  }

  public boolean getWantsGripToggle() {
    return this.getRawButtonPressed(5);
  }

  public boolean getWantsHighConeScore() {
    return this.getHatUpPressed();
  }

  public boolean getWantsDoubleSubstation() {
    return this.getHatRight();
  }

  public boolean getWantsGroundPickup() {
    return this.getHatDown();
  }
}
