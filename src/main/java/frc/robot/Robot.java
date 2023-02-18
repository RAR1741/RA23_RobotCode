package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.Arm.State;

public class Robot extends TimedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  // private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final Arm m_arm = Arm.getInstance();

  // private UsbCamera mCamera;

  // private final Timer m_stoppedTimer = new Timer();

  private final Field2d m_field = new Field2d();

  @Override
  public void robotInit() {
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

    // m_allSubsystems.add(m_swerve);
    m_allSubsystems.add(m_arm);
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

    double rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis()) *
        Constants.Drivetrain.k_maxAngularSpeed;

    // m_swerve.drive(mDriverController.getForwardAxis(),
    // mDriverController.getStrafeAxis(),
    // 0, true);

    // mDrive.slowMode(mDriverController.getWantsSlowMode());

    // m_swerve.drive(xSpeed, ySpeed, 0, true);
    // if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
    // m_stoppedTimer.start();
    // } else {
    // m_stoppedTimer.reset();
    // m_stoppedTimer.stop();
    // }

    // if (m_stoppedTimer.hasElapsed(1.0)) {
    // m_swerve.pointDirection(1.0, 0.0, 0.0, false);
    // } else {
    // m_swerve.drive(xSpeed, ySpeed, rot, true);
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

    // if (mDriverController.getWantsResetGyro()) {
    // m_swerve.resetGyro();
    // }

    if (m_operatorController.getWantsDefaultState()) {
      m_arm.setState(State.DEFAULT);
      System.out.println("Going to default");
    }

    if(m_operatorController.getHatUp()) {
      m_arm.lowerStates(m_operatorController.getWantsMaxMovement());
      System.out.println("Lowering");
    }
    
    if(m_operatorController.getHatDown()) {
      m_arm.raiseStates(m_operatorController.getWantsMaxMovement());
      System.out.println("Raising");
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
    if (m_driverController.getRawButtonPressed(3)) {
      Preferences.setDouble("shoulderAngle", 0);
    } else if (m_driverController.getRawButtonPressed(4)) {
      Preferences.setDouble("shoulderAngle", 90);
    } else if (m_driverController.getRawButtonPressed(1)) {
      Preferences.setDouble("shoulderAngle", 180);
    }
    updateSim();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    // mDrive.simulationPeriodic();
    // m_field.setRobotPose(m_swerve.getPose());
  }
}
