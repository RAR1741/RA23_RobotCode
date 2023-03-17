package frc.robot.autonomous.tasks;

import frc.robot.subsystems.drivetrain.SwerveDrive;

public class PointForwardsTask extends Task {
  private SwerveDrive m_swerve;

  public PointForwardsTask(double distance, double xSpeed) {
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
    m_swerve.pointModules(0, 0, 0, true);
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
  }
}
