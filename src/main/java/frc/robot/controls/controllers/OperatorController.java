package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsGroundPosition() {
    return this.getRawButton(1);
  }
}
