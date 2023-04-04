package frc.robot.autonomous.tasks;

import frc.robot.subsystems.drivetrain.SwerveDrive;

public class BrakeTask extends Task {
  private SwerveDrive m_swerve;
  private boolean m_brake;

  public BrakeTask(boolean brake) {
    m_brake = brake;
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
    if (m_brake) {
      m_swerve.brakeOn();
    } else {
      m_swerve.brakeOff();
    }

  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    m_swerve.drive(0, 0, 0, false);
  }
}
