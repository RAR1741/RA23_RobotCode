package frc.robot.controls.controllers;

import frc.robot.logging.Loggable;
import frc.robot.logging.Logger;

public class DriverController extends FilteredController implements Loggable {
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
    return this.getFilteredAxis(0);
  }

  public double getTurnAxis() {
    return this.getFilteredAxis(2);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > kTriggerActivationThreshold;
  }

  @Override
  public void logHeaders(Logger logger) {
    logger.addHeader("DriverController");
    logger.addHeader("DriverController/Port");
    logger.addHeader("FowardAxis");
    logger.addHeader("StrafeAxis");
    logger.addHeader("TurnAxis");
    // logger.addHeader("SlowMode");
  }

  @Override
  public void logData(Logger logger) {
    logger.addData("DriverController", getName());
    logger.addData("DriverController/Port", getPort());
    logger.addData("FowardAxis", getForwardAxis()); // TODO: Maybe add Axis ID's to Constants class
    logger.addData("FowardAxis", getStrafeAxis());
    logger.addData("FowardAxis", getTurnAxis());
    // logger.addData("SlowMode", getWantsSlowMode());
  }

  // Intake
  /*public boolean getWantsIntakeOpen() {
    return this.getLeftBumper();
  }

  public boolean getWantsIntakeClose() {
    return this.getRightBumper();
  }*/
}
