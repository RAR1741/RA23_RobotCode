package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.RobotBase;

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

  public double getTurnAxis() {
    if (RobotBase.isReal()) {
      // Righthand joystick
      return this.getFilteredAxis(4);

      // Lefthand joysick
      // return this.getFilteredAxis(0);
    } else {
      return this.getFilteredAxis(2);
    }
  }

  // Intake
  public boolean getWantsIntakeOpen() {
    return this.getLeftBumper();
  }

  public boolean getWantsIntakeClose() {
    return this.getRightBumper();
  }

  // Elevator
  public boolean getWantsExtensionStow() {
    return this.getRawButton(1);
  }

  public boolean getWantsExtensionMidGoal() {
    return this.getRawButton(3);
  }

  public boolean getWantsExtensionHighGoal() {
    return this.getRawButton(4);
  }

  public boolean getWantsAutoScore() {
    return this.getRawButton(2);
  }

  public boolean getWantsExtend() {
    return false;
    // return this.getRawButton(4);
  }

  public boolean getWantsRetract() {
    return false;
    // return this.getRawButton(3);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > kTriggerActivationThreshold;
  }

  // public boolean getWantsRaise() {
  // return this.getFilteredAxis(3) > kTriggerActivationThreshold;
  // // return this.getRawButton(2);
  // }

  // public boolean getWantsLower() {
  // return this.getFilteredAxis(2) > kTriggerActivationThreshold;
  // // return this.getRawButton(1);
  // }

}

// a is pivot up (should be Y)
// b is extend down (should be A)
// x is extend up (should be B)
// y is pivot down (should be X)
// set pivot to brake
// set extend to brake

// public boolean getWantsExtend() {
// return this.getBButton();
// }

// public boolean getWantsRetract() {
// return this.getAButton();
// }

// public boolean getWantsRaise() {
// return this.getYButton();
// }

// public boolean getWantsLower() {
// return this.getXButton();
// }
