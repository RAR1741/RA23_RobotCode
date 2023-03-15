package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.Timer;

public class WaitTask extends Task {
  private Timer m_runningTimer = new Timer();
  private double m_targetTime;

  public WaitTask(double timeSeconds) {
    m_targetTime = timeSeconds;
  }

  @Override
  public void start() {
    m_runningTimer.start();
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return m_runningTimer.get() >= m_targetTime;
  }
}
