package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveForwardTask extends Task {
  private SwerveDrive m_swerve;
  private double m_targetDistance;
  private double m_xSpeed;

  private Timer m_runningTimer = new Timer();
  private double m_lastTime = 0;

  private Pose2d m_targetPose;

  public DriveForwardTask(double distance, double xSpeed) {
    m_swerve = SwerveDrive.getInstance();
    m_targetDistance = distance;
    m_xSpeed = xSpeed;
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    Pose2d currentPose = m_swerve.getPose();
    double x = currentPose.getX() + m_targetDistance;
    m_targetPose = new Pose2d(x, currentPose.getY(), currentPose.getRotation());
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
    // return m_runningTimer.get() > m_targetDistance / m_xSpeed;
    return m_swerve.getPose().getX() >= m_targetPose.getX();
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto done driving", false);
    m_swerve.drive(0, 0, 0, false);
  }
}
