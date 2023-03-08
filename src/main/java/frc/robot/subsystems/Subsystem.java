package frc.robot.subsystems;

public abstract class Subsystem {
  public void initializeLog() {
  }

  public void zeroSensors() {
  }

  public abstract void periodic();

  public abstract void stop();

  public abstract void writePeriodicOutputs();

  public abstract void outputTelemetry();

  public abstract void writeToLog();
}
