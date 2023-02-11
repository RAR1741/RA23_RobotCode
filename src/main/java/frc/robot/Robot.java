package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.controllers.DriverController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Robot extends TimedRobot {
  private final DriverController mDriverController = new DriverController(0, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter mXRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter mYRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter mRotRateLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private List<Subsystem> mAllSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();

  // private UsbCamera mCamera;

  // private final Timer m_stoppedTimer = new Timer();

  private final Field2d mField = new Field2d();

  @Override
  public void robotInit() {
    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", mField);

    // Camera server
    /*
     * if (RobotBase.isReal()) {
     * mCamera = CameraServer.startAutomaticCapture();
     * mCamera.setFPS(30);
     * mCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
     * }
     */

    mAllSubsystems.add(m_swerve);
  }

  @Override
  public void robotPeriodic() {
    mAllSubsystems.forEach(subsystem -> subsystem.periodic());
  }

  @Override
  public void autonomousInit() {
    m_swerve.resetKinematics();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_swerve.resetKinematics();
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = mXRateLimiter.calculate(mDriverController.getForwardAxis())
        * SwerveDrive.kMaxSpeed;

    double ySpeed = mYRateLimiter.calculate(mDriverController.getStrafeAxis())
        * SwerveDrive.kMaxSpeed;

    double rot = mRotRateLimiter.calculate(mDriverController.getTurnAxis()) *
        SwerveDrive.kMaxAngularSpeed;
    
    // if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
    //   m_stoppedTimer.start();
    // } else {
    //   m_stoppedTimer.reset();
    //   m_stoppedTimer.stop();
    // }

    // if (m_stoppedTimer.hasElapsed(1.0)) {
    //   m_swerve.pointDirection(1.0, 0.0, 0.0, false);
    // } else {
    m_swerve.drive(xSpeed, ySpeed, rot, true);
    // }

    // m_swerve.drive(0.3, 0, 0, false);
    // m_swerve.drive(0, 0.1, 0, false);
    // m_swerve.drive(0, 0, 0.1, false);

    // Intake controls
    /*
     * if (mDriverController.getWantsIntakeOpen()) {
     * // m_intake.open();
     * } else if (mDriverController.getWantsIntakeClose()) {
     * // m_intake.close();
     * }
     */

    if (mDriverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    mAllSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    mAllSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    mAllSubsystems.forEach(subsystem -> subsystem.writeToLog());

    mDriverController.outputTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

  @Override
  public void disabledInit() {
    mAllSubsystems.forEach(subsystem -> subsystem.stop());
  }

  @Override
  public void disabledPeriodic() {
    // Stop the robot when disabled.
    m_swerve.drive(0.0, 0.0, 0.0, true);

    updateSim();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    // mDrive.simulationPeriodic();
    mField.setRobotPose(m_swerve.getPose());
  }
}
