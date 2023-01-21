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
  private final SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter mRotLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private List<Subsystem> mAllSubsystems = new ArrayList<>();
  private final SwerveDrive mSwerve = SwerveDrive.getInstance();

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

    mAllSubsystems.add(mSwerve);
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
    // double xSpeed = -mSpeedLimiter.calculate(mDriverController.getForwardAxis())
    // * Drivetrain.kMaxSpeed;

    // mDrive.slowMode(mDriverController.getWantsSlowMode());

    // double rot = -mRotLimiter.calculate(mDriverController.getTurnAxis()) *
    // Drivetrain.kMaxAngularSpeed;

    // mSwerve.drive(mDriverController.getForwardAxis(),mDriverController.getStrafeAxis(),
    // mDriverController.getTurnAxis(),true);
    // mSwerve.drive(6, 0, 0, false);
    mSwerve.drive(0, 6, 0, false);

    // // Intake controls
    /*
     * if (mDriverController.getWantsIntakeOpen()) {
     * // m_intake.open();
     * } else if (mDriverController.getWantsIntakeClose()) {
     * // m_intake.close();
     * }
     */

    mAllSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    mAllSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    mAllSubsystems.forEach(subsystem -> subsystem.writeToLog());
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
    // mDrive.drive(0.0, 0.0);

    updateSim();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    // mDrive.simulationPeriodic();
    // mField.setRobotPose(mDrive.getPose());
  }
}
