package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.controls.controllers.DriverController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Robot extends LoggedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(3);

  private Logger m_logger;

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();

  // private UsbCamera mCamera;

  // private final Timer m_stoppedTimer = new Timer();

  private final Field2d m_field = new Field2d();

  @Override
  public void robotInit() {
    // Initialize on-board logging
    m_logger = Logger.getInstance();
    
    if (isReal()) {
      m_logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); //TODO: Find out what the value is, rn it goes to a USB stick potentially plugged in tp the RoboRIO
      m_logger.addDataReceiver(new NT4Publisher()); // Publish to NT
    } else {
      m_logger.addDataReceiver(new WPILOGWriter(""));
      m_logger.addDataReceiver(new NT4Publisher());
    }

    m_logger.start();

    System.out.println("Logging initialized. Fard."); //It logs the console :P
    // DataLogManager.log("Logging initialized. Fard."); // :(

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    // Camera server
    /*
     * if (RobotBase.isReal()) {
     * mCamera = CameraServer.startAutomaticCapture();
     * mCamera.setFPS(30);
     * mCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
     * }
     */

    m_allSubsystems.add(m_swerve);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis())
        * Constants.Drivetrain.k_maxSpeed;

    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis())
        * Constants.Drivetrain.k_maxSpeed;

    double rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis())
        * Constants.Drivetrain.k_maxAngularSpeed;
    
    if(m_driverController.getWantsSlowMode()) {
      xSpeed *= Constants.Drivetrain.k_slowScaler;
      ySpeed *= Constants.Drivetrain.k_slowScaler;
    }

    // if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
    // m_stoppedTimer.start();
    // } else {
    // m_stoppedTimer.reset();
    // m_stoppedTimer.stop();
    // }

    // if (m_stoppedTimer.hasElapsed(1.0)) {
    // m_swerve.pointModules(1.0, 0.0, 0.0, false);
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

    if (m_driverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    m_driverController.outputTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

  @Override
  public void disabledInit() {
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
  }

  @Override
  public void disabledPeriodic() {
    updateSim();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    // mDrive.simulationPeriodic();
    m_field.setRobotPose(m_swerve.getPose());
  }
}
