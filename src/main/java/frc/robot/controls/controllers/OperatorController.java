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

  public boolean getWantsCycleStateUp() {
    return this.getHatDown();
  }

  public boolean getWantsCycleStateDown() {
    return this.getHatUp();
  }

  public boolean getWantsMaxMovement() {
    return this.getRawButton(3);
  }

  public boolean getWantsGroundPosition() {
    return this.getRawButton(1);
  }

public boolean getWantsGripToggle() {
    return this.getRawButton(5);
}
}
