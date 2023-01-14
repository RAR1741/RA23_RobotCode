package frc.robot.subsystems;

public abstract class Subsystem {
  public void writeToLog() {
  }

  public void zeroSensors() {
  }

  public abstract void periodic();

  public abstract void stop();

  public abstract void writePeriodicOutputs();

  public abstract void outputTelemetry();
}
