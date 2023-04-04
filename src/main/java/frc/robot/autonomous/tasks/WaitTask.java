package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class WaitTask extends Task {
  private Timer m_runningTimer = new Timer();
  private double m_targetTime;

  private SwerveDrive m_swerve = SwerveDrive.getInstance();

  public WaitTask(double timeSeconds) {
    m_targetTime = timeSeconds;
  }

  @Override
  public void start() {
    m_runningTimer.start();
  }

  @Override
  public void update() {
    m_swerve.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return m_runningTimer.get() >= m_targetTime;
  }
}
