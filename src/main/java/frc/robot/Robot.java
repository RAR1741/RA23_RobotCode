package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.cscore.UsbCamera;
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
  private final SlewRateLimiter mXSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter mYSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter mRotLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private List<Subsystem> mAllSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();

  private UsbCamera mCamera;

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
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = mXSpeedLimiter.calculate(mDriverController.getForwardAxis())
        * SwerveDrive.kMaxSpeed;

    double ySpeed = mYSpeedLimiter.calculate(mDriverController.getStrafeAxis())
        * SwerveDrive.kMaxSpeed;

    double rot = mRotLimiter.calculate(mDriverController.getTurnAxis()) *
        SwerveDrive.kMaxAngularSpeed;

    // m_swerve.drive(mDriverController.getForwardAxis(),
    // mDriverController.getStrafeAxis(),
    // 0, true);

    // mDrive.slowMode(mDriverController.getWantsSlowMode());

    // m_swerve.drive(xSpeed, ySpeed, 0, true);
    m_swerve.drive(xSpeed, ySpeed, rot, true);

    // m_swerve.drive(0.3, 0, 0, false);
    // m_swerve.drive(0, 0.1, 0, false);
    // m_swerve.drive(0, 0, 0.1, false);
    // m_swerve.drive(0.1, 0, 0.1, false);

    // // Intake controls
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
