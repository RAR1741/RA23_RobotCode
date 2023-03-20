package frc.robot.autonomous.tasks;

import frc.robot.subsystems.drivetrain.SwerveDrive;

public class PointModulesInwardTask extends Task {
  private SwerveDrive m_swerve;

  public PointModulesInwardTask() {
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
    m_swerve.pointInwards();
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
