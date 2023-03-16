package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
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
    double x = currentPose.getX() + (m_targetDistance * Math.cos(currentPose.getRotation().getRadians()));
    double y = currentPose.getY() + (m_targetDistance * Math.sin(currentPose.getRotation().getRadians()));
    m_targetPose = new Pose2d(x, y, currentPose.getRotation());
  }

  @Override
  public void update() {
    m_swerve.drive(m_xSpeed, 0, 0, true);

    if (!RobotBase.isReal()) {
      // This simulates the robot driving in the positive x direction
      Pose2d currentPose = m_swerve.getPose();

      // Move "forward", based on the robot's current rotation
      double newX = currentPose.getX()
          + m_xSpeed * (m_runningTimer.get() - m_lastTime) * Math.cos(currentPose.getRotation().getRadians());
      double newY = currentPose.getY()
          + m_xSpeed * (m_runningTimer.get() - m_lastTime) * Math.sin(currentPose.getRotation().getRadians());

      Pose2d newPose = new Pose2d(
          newX,
          newY,
          currentPose.getRotation());

      m_swerve.setPose(newPose);
      m_lastTime = m_runningTimer.get();
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d relativePose = m_swerve.getPose().relativeTo(m_targetPose);
    return Math.abs(relativePose.getX()) <= 0.1 && Math.abs(relativePose.getY()) <= 0.1;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto done driving", false);
    m_swerve.drive(0, 0, 0, false);
  }
}
