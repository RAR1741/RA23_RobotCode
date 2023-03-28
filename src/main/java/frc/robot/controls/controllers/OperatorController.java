package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public double getArmHorizontalChange(double strength) {
    return -this.getFilteredAxis(0) * strength;
  }

  public double getArmVerticalChange(double strength) {
    return this.getFilteredAxis(5) * -strength;
  }

  public boolean getWantsGripToggle() {
    return this.getRawButtonPressed(5);
  }

  public boolean getWantsColorCycle() {
    return false;
    // return this.getRawButtonPressed(7);
  }

  public boolean getWantsRobotFrontInverted() {
    // return false;
    return this.getRawButtonPressed(7);
  }

  public boolean getWantsDefaultState() {
    return this.getRawButtonPressed(2);
  }

  public boolean getWantsDoubleSubstation() {
    return this.getRawButtonPressed(4);
  }

  public boolean getWantsGroundPickup() {
    return this.getRawButtonPressed(1);
  }

  public boolean getWantsGroundScore() {
    return this.getRawButtonPressed(3);
  }

  public boolean getWantsHighConeScore() {
    return this.getHatUpPressed();
  }

  public boolean getWantsMidConeScore() {
    return this.getHatRightPressed();
  }

  public boolean getWantsHighCubeScore() {
    return this.getHatLeftPressed();
  }

  public boolean getWantsMidCubeScore() {
    return this.getHatDownPressed();
  }

  public boolean getWantsElbowChange() {
    return this.getRawButton(8);
  }

  public boolean getWantsWristRotate() {
    return this.getRawButtonPressed(6);
  }

  public boolean getWantsWristCorrectionClockwise() {
    return this.getFilteredAxis(3) > 0.5;
  }

  public boolean getWantsWristCorrectionCounterClockwise() {
    return this.getFilteredAxis(2) > 0.5;
  }
}
