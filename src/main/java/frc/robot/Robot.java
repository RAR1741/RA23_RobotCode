package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
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
  public final SwerveDrive m_swerve = SwerveDrive.getInstance();
  public final Arm m_arm = Arm.getInstance();
  private Task m_currentTask;
  private AutoRunner m_autoRunner;

  // The mere instantiation of this object will cause the compressor to start
  // running. We don't need to do anything else with it, so we'll suppress the
  // warning.
  @SuppressWarnings("unused")
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  // private int test_state = 1;

  @SuppressWarnings("unused")
  private UsbCamera m_camera;

  // Auto things
  AutoChooser m_autoChooser = new AutoChooser();

  private final Field m_field = Field.getInstance();

  @Override
  public void robotInit() {
    // Initialize on-board logging
    DataLogManager.start();
    DataLogManager.log("Logging initialized. Fard.");

    // Start the PathPlanner server
    PathPlannerServer.startServer(5811);

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    m_arm.setGripper(true);
    m_arm.clearPIDAccumulation();

    // Camera server
    m_camera = CameraServer.startAutomaticCapture();

    m_allSubsystems.add(m_swerve);
    m_allSubsystems.add(m_arm);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    updateSim();
  }

  @Override
  public void autonomousInit() {
    m_swerve.brakeOff();

    m_autoRunner = AutoRunner.getInstance();
    // TODO: Change this to use the AutoChooser
    m_autoRunner.setAutoMode(AutoRunner.AutoMode.RED_CUBE_BALANCE);
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_arm.runTrajectory();

    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    m_swerve.brakeOff();
    m_swerve.drive(0, 0, 0, false);
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis())
        * Constants.Drivetrain.k_maxSpeed;

    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis())
        * Constants.Drivetrain.k_maxSpeed;

    double rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis()) *
        Constants.Drivetrain.k_maxAngularSpeed;

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = Constants.Drivetrain.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - Constants.Drivetrain.k_slowScaler));

    // boostScaler should scale between 1 and k_boostScaler
    double boostScaler = 1 + (m_driverController.getBoostScaler() * (Constants.Drivetrain.k_boostScaler - 1));

    xSpeed *= slowScaler * boostScaler;
    ySpeed *= slowScaler * boostScaler;

    m_swerve.drive(xSpeed, ySpeed, rot, true);

    if (m_driverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    if (m_driverController.getWantsGripToggle() || m_operatorController.getWantsGripToggle()) {
      m_arm.setGripper(!m_arm.getGripperEngaged());
    }

    if (!m_arm.runTrajectory()) {
      m_arm.adjustPosition(m_operatorController.getArmHorizontalChange(0.5),
          m_operatorController.getArmVerticalChange(0.5));
    }

    if (m_operatorController.getRawButtonPressed(6)) {
      m_arm.rotateWrist();
    }

    if (m_operatorController.getWantsDefaultState()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.HOME.getPose());
    }

    if (m_operatorController.getWantsDoubleSubstation()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose());
    }

    if (m_operatorController.getWantsGroundPickup()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.FLOOR_PICKUP.getPose());
    }

    if (m_operatorController.getWantsGroundScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.FLOOR_SCORE.getPose());
    }

    if (m_operatorController.getWantsHighConeScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.SCORE_HIGH_CONE.getPose());
    }

    if (m_operatorController.getWantsMidConeScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.SCORE_MID_CONE.getPose());
    }

    if (m_operatorController.getWantsHighCubeScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose());
    }

    if (m_operatorController.getWantsMidCubeScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.SCORE_MID_CUBE.getPose());
    }

    if (m_driverController.getWantsGripToggle() || m_operatorController.getWantsGripToggle()) {
      m_arm.setGripper(!m_arm.getGripperEngaged());
    }

    /*
     * Ground pickup A
     * Station Y
     * Home B
     * Ground place X
     * High cone hat up
     * High cube hat left
     * Medium cone hat right
     * Medium cube hat down
     */

    m_driverController.outputTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

  @Override
  public void disabledInit() {
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
    m_swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void disabledPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());

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

    if (!m_arm.runTrajectory()) {
      m_arm.adjustPosition(m_operatorController.getArmHorizontalChange(0.5),
          m_operatorController.getArmVerticalChange(0.5));
    }

    // Preferences.setDouble("targetX", targetX +=
    // m_operatorController.getArmHorizontalChange(0.5));
    // Preferences.setDouble("targetY", targetY +=
    // m_operatorController.getArmVerticalChange(0.5));

    // if(startTraj == 1) {
    // Preferences.setDouble("startTraj", 2);

    // /*double[] targetAngles = m_arm.setArmPosition(targetX, targetY);
    // SmartDashboard.putNumberArray("CalcXY Double Check",
    // m_arm.calcXY(targetAngles[0], targetAngles[1]));

    // ArmPose target = new ArmPose(trajEndX,trajEndY,new Rotation2d(0));*/
    // ArmPose target = new ArmPose(targetX, targetY, new Rotation2d(0));

    // m_arm.generateTrajectoryToPose(target);
    // } else if(startTraj == 2) {
    // Preferences.setDouble("startTraj", m_arm.runTrajectory() ? 2 : 0);
    // } else {
    // double[] targetAngles = m_arm.setArmPosition(targetX, targetY);
    // SmartDashboard.putNumberArray("CalcXY Double Check",
    // m_arm.calcXY(targetAngles[0], targetAngles[1]));
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
    }

    if (m_operatorController.getWantsDoubleSubstation()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose());
    }

    if (m_operatorController.getWantsHighConeScore()) {
      m_arm.generateTrajectoryToPose(Constants.Arm.Preset.SCORE_HIGH_CONE.getPose());
    }

    if (m_driverController.getWantsGripToggle() || m_operatorController.getWantsGripToggle()) {
      m_arm.setGripper(!m_arm.getGripperEngaged());
    }

    m_arm.outputTelemetry();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_swerve.getPose());
  }
}
