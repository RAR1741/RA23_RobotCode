package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.logging.Loggable;
import frc.robot.logging.Logger;

public class DriverController extends FilteredController implements Loggable {
  private String m_smartDashboardKey = "DriverInput/";
  
  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  private final double k_triggerActivationThreshold = 0.5;

  // Drive
  public double getForwardAxis() {
    return -this.getFilteredAxis(1);
  }

  public double getStrafeAxis() {
    return -this.getFilteredAxis(0);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(4);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  public boolean getWantsResetGyro() {
    return this.getRawButton(4);
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
  /*
   * public boolean getWantsIntakeOpen() {
   * return this.getLeftBumper();
   * }
   *
   * public boolean getWantsIntakeClose() {
   * return this.getRightBumper();
   * }
   */

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Strafe", getStrafeAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
