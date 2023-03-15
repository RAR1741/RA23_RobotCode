package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveForwardTask extends Task {
  private SwerveDrive m_swerve;
  private double m_targetDistance;
  private double m_xSpeed;

  private Timer m_runningTimer = new Timer();
  private double m_lastTime = 0;

  public DriveForwardTask(double distance, double xSpeed) {
    m_swerve = SwerveDrive.getInstance();
    m_targetDistance = distance;
    m_xSpeed = xSpeed;
  }

  @Override
  public void start() {
    m_runningTimer.start();
  }

  @Override
  public void update() {
    m_swerve.drive(m_xSpeed, 0, 0, true);

    if (!RobotBase.isReal()) {
      // This simulates the robot driving in the positive x direction
      Pose2d currentPose = m_swerve.getPose();
      Pose2d newPose = new Pose2d(
          currentPose.getX() + m_xSpeed * (m_runningTimer.get() - m_lastTime),
          currentPose.getY(),
          currentPose.getRotation());

      m_swerve.setPose(newPose);
      m_lastTime = m_runningTimer.get();
    }
  }

  @Override
  public boolean isFinished() {
    return m_swerve.getPose().getX() >= m_targetDistance;
  }

  @Override
  public void done() {
    m_swerve.drive(0, 0, 0, false);
  }
}
