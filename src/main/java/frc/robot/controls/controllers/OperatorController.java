package frc.robot.controls.controllers;

import frc.robot.logging.Loggable;
import frc.robot.logging.Logger;

public class OperatorController extends FilteredController implements Loggable {
  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsGroundPosition() {
    return this.getRawButton(1);
  }

  @Override
  public void logHeaders(Logger logger) {
    logger.addHeader("OperatorController");
    logger.addHeader("OperatorController/Port");
    logger.addHeader("GroundPosition");
  }

  @Override
  public void logData(Logger logger) {
    logger.addData("OperatorController", getName());
    logger.addData("OperatorController/Port", getPort());
    logger.addData("GroundPosition", getWantsGroundPosition());
  }
}
