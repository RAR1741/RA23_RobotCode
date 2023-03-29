package frc.robot.autonomous.tasks;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private SwerveDrive m_swerve;
  private PathPlannerTrajectory m_autoPath;
  private boolean m_isFinished = false;

  private final Timer m_runningTimer = new Timer();
  private PPHolonomicDriveController m_driveController;

  public DriveTrajectoryTask(String pathName, double maxSpeed, double maxAcceleration) {
    try {
      m_autoPath = PathPlanner.loadPath(pathName, new PathConstraints(maxSpeed, maxAcceleration));
    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
      m_isFinished = true;
    }

    m_swerve = SwerveDrive.getInstance();

    m_driveController = new PPHolonomicDriveController(
        new PIDController(0.75, 0, 0),
        new PIDController(0.3, 0.005, 0),
        new PIDController(1.0, 0, 0));
  }

  @Override
  public void start() {
    // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
    // m_autoPath = AutoModeBase.transformTrajectoryForAlliance(m_autoPath);
    // }

    Pose2d startingPosition = m_autoPath.getInitialPose();
    m_swerve.setGyroAngleAdjustment(startingPosition.getRotation().getDegrees());
    m_swerve.resetOdometry(startingPosition);

    m_runningTimer.reset();
    m_runningTimer.start();

    m_swerve.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().toString(), false);
  }

  @Override
  public void update() {
    PathPlannerState autoState = (PathPlannerState) m_autoPath.sample(m_runningTimer.get());

    ChassisSpeeds chassisSpeeds = m_driveController.calculate(m_swerve.getPose(), autoState);

    m_swerve.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond, // This is probably wrong...
        false); // And we probably want to change this to true

    m_isFinished |= m_runningTimer.get() >= m_autoPath.getTotalTimeSeconds();
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      PathPlannerState autoState = (PathPlannerState) m_autoPath.sample(m_runningTimer.get());

      Pose2d targetPose2d = new Pose2d(
          autoState.poseMeters.getX(),
          autoState.poseMeters.getY(),
          autoState.holonomicRotation);

      m_swerve.setPose(targetPose2d);
    }
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto trajectory done", false);
    m_swerve.drive(0, 0, 0, true);
  }
}
