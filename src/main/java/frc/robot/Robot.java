package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Robot extends TimedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final Arm m_arm = Arm.getInstance();

  // The mere instantiation of this object will cause the compressor to start
  // running. We don't need to do anything else with it, so we'll suppress the
  // warning.
  @SuppressWarnings("unused")
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  // private int test_state = 1;

  // private UsbCamera mCamera;

  private final Timer m_stoppedTimer = new Timer();

  PathPlannerTrajectory autoPath;

  private final Field2d m_field = new Field2d();

  @Override
  public void robotInit() {
    // Initialize on-board logging
    DataLogManager.start();
    DataLogManager.log("Logging initialized. Fard.");

    // Start the PathPlanner server
    PathPlannerServer.startServer(5811);

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    m_arm.setGripper(false);
    m_arm.clearPIDAccumulation();

    // Camera server
    /*
     * if (RobotBase.isReal()) {
     * mCamera = CameraServer.startAutomaticCapture();
     * mCamera.setFPS(30);
     * mCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
     * }
     */

    m_allSubsystems.add(m_swerve);
    m_allSubsystems.add(m_arm);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
  }

  @Override
  public void autonomousInit() {
    autoPath = PathPlanner.loadPath("BlueDefault",
        new PathConstraints(Constants.Drivetrain.k_maxSpeed, Constants.Drivetrain.k_maxAngularSpeed));

    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("scoreHigh", new PrintCommand("Passed marker 1"));

    m_stoppedTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    PathPlannerState autoState = (PathPlannerState) autoPath.sample(m_stoppedTimer.get());

    // Print the velocity at the sampled time
    System.out.println(autoState.velocityMetersPerSecond);

    // m_swerve.drive(xSpeed, ySpeed, rot, true);
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

    if (m_driverController.getWantsSlowMode()) {
      xSpeed *= Constants.Drivetrain.k_slowScaler;
      ySpeed *= Constants.Drivetrain.k_slowScaler;
    }

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

    if (m_driverController.getWantsGripToggle() || m_operatorController.getWantsGripToggle()) {
      m_arm.setGripper(!m_arm.getGripperEngaged());
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
    // if (m_driverController.getRawButtonPressed(3)) {
    // Preferences.setDouble("shoulderAngle", 0);
    // } else if (m_driverController.getRawButtonPressed(4)) {
    // Preferences.setDouble("shoulderAngle", 90);
    // } else if (m_driverController.getRawButtonPressed(1)) {
    // Preferences.setDouble("shoulderAngle", 180);
    // }
    updateSim();
  }

  @Override
  public void testInit() {
    if (!Preferences.containsKey("targetX")) {
      Preferences.setDouble("targetX", 0);
    }

    if (!Preferences.containsKey("targetY")) {
      Preferences.setDouble("targetY", Constants.Arm.k_homeHeight);
    }

    if (!Preferences.containsKey("wristAngle")) {
      Preferences.setDouble("wristAngle", 0);
    }
  }

  @Override
  public void testPeriodic() {
    m_swerve.drive(0, 0, 0, false);

    // SmartDashboard.putNumberArray("Arm Values", m_arm.calcAngles(posX, posY));
    // double targetX = Preferences.getDouble("targetX", 0);
    // double targetY = Preferences.getDouble("targetY", 11.5);

    // // double wristAngle = Preferences.getDouble("wristAngle", 0);

    // double startTraj = Preferences.getDouble("startTraj", 0);

    if(!m_arm.runTrajectory()) {
      m_arm.adjustPosition(m_operatorController.getArmHorizontalChange(0.5), m_operatorController.getArmVerticalChange(0.5));
    }

    //Preferences.setDouble("targetX", targetX += m_operatorController.getArmHorizontalChange(0.5));
    //Preferences.setDouble("targetY", targetY += m_operatorController.getArmVerticalChange(0.5));

    // if(startTraj == 1) {
    //   Preferences.setDouble("startTraj", 2);

    //   /*double[] targetAngles = m_arm.setArmPosition(targetX, targetY);
    //   SmartDashboard.putNumberArray("CalcXY Double Check", m_arm.calcXY(targetAngles[0], targetAngles[1]));

    //   ArmPose target = new ArmPose(trajEndX,trajEndY,new Rotation2d(0));*/
    //   ArmPose target = new ArmPose(targetX, targetY, new Rotation2d(0));

    //   m_arm.generateTrajectoryToPose(target);
    //   m_arm.startTrajectory();
    // } else if(startTraj == 2) {
    //   Preferences.setDouble("startTraj", m_arm.runTrajectory() ? 2 : 0);
    // } else {
    //   double[] targetAngles = m_arm.setArmPosition(targetX, targetY);
    //   SmartDashboard.putNumberArray("CalcXY Double Check", m_arm.calcXY(targetAngles[0], targetAngles[1]));
    // }

    // m_arm.setWristAngle(wristAngle);

    // switch(test_state) {
    // case 0:
    // m_arm.manual(m_operatorController.getRawAxis(5) * 0.2, 0, 0);
    // break;
    // case 1:
    // m_arm.manual(0, m_operatorController.getRawAxis(5) * 0.2, 0);
    // break;
    // case 2:
    // m_arm.manual(0, 0, m_operatorController.getRawAxis(5) * 0.2);
    // break;
    // default:
    // break;
    // }

    // if (m_operatorController.getRawButtonPressed(3)) {
    // test_state = test_state == 2 ? 0 : test_state + 1;
    // }

    if (m_operatorController.getRawButtonPressed(6)) {
      m_arm.rotateWrist();
    }

    if (m_operatorController.getWantsDefaultState()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.HOME.getPose());
      m_arm.startTrajectory();
    }

    if (m_operatorController.getWantsDoubleSubstation()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose());
      m_arm.startTrajectory();
    }

    if (m_operatorController.getWantsHighConeScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.SCORE_HIGH_CONE.getPose());
      m_arm.startTrajectory();
    }

    if (m_driverController.getWantsGripToggle() || m_operatorController.getWantsGripToggle()) {
      m_arm.setGripper(!m_arm.getGripperEngaged());
    }

    if (m_driverController.getRawButtonPressed(3)) {
      m_arm.rezero();
    }

    m_arm.outputTelemetry();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    // mDrive.simulationPeriodic();
    // m_field.setRobotPose(m_swerve.getPose());
  }
}
