package frc.robot.autonomous.tasks;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoBalanceTask extends Task {
  private SwerveDrive m_swerve;
  private AHRS m_gyro;

  private double m_tolerance = 5.0;
  private double m_maxSpeed = 0.325;

  @Override
  public void start() {
    m_swerve = SwerveDrive.getInstance();
    m_gyro = m_swerve.getGyro();
  }

  public void update() {
    double pitch = m_gyro.getPitch();
    double speed = 0;

    if (pitch < 0) {
      speed = m_maxSpeed;
    } else if (pitch > 0) {
      speed = -m_maxSpeed;
    }

    double currentHeading = m_swerve.getPose().getRotation().getRadians();
    double xSpeed = speed * Math.cos(currentHeading);
    double ySpeed = speed * Math.sin(currentHeading);

    m_swerve.drive(xSpeed, ySpeed, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_gyro.getPitch()) <= m_tolerance;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto balance done", false);
    m_swerve.drive(0, 0, 0, true);
  }
}
