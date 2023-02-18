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

  /**
   * 
   * @param direction 0 = UP, 90 = RIGHT, 180 = DOWN, 270 = LEFT
   * @return If the specified direction is pressed within a 45° threshold
   */
  public boolean getHatState(int direction) {
    int dPadValue = this.getPOV();
    return (dPadValue == direction) || (dPadValue == (direction + 45) % 360) || (dPadValue == (direction + 315) % 360);
  }

  public boolean getWantsMaxMovement() {
    return this.getRawButton(3);
  }

  public boolean getWantsGroundPosition() {
    return this.getRawButton(1);
  }
}
