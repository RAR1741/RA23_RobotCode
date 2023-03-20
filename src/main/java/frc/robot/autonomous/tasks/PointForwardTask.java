package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class PointForwardTask extends Task {
  private SwerveDrive m_swerve;

  public PointForwardTask() {
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
    m_swerve.pointModules(1, 0, 0, true);
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
    DriverStation.reportWarning("Auto point forward done", false);
  }
}
