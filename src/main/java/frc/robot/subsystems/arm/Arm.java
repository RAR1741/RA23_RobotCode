package frc.robot.subsystems.arm;

import java.util.ArrayList;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.simulation.ArmSim;
import frc.robot.subsystems.Subsystem;

public class Arm extends Subsystem {
  private static Arm m_arm = null;
  private static ArmSim m_armSim = null;

  private final String m_smartDashboardKey = "Arm/";

  private static final double k_shoulderMotorP = 0.1;
  private static final double k_shoulderMotorI = 0.02;
  private static final double k_shoulderMotorD = 0.0;

  private static final double k_elbowMotorP = 0.125;
  private static final double k_elbowMotorI = 0.0;
  private static final double k_elbowMotorD = 0.0;

  private static final double k_wristMotorP = 0.05;
  private static final double k_wristMotorI = 0.025;
  private static final double k_wristMotorD = 0.0;

  // The units for Trajectories and Pose2d's are in meters, but
  // we're going to pretend it's in inches.  As long as we put in inches
  // and expected inches to come out, it doesn't really matter
  private static final double k_maxTrajectorySpeed = 5; // 5 inches per second
  private static final double k_maxTrajectoryAcceleration = 5; // 5 in/s^2

  // TODO: Update for actual robot
  // distance per pulse = (angle per revolution) / (pulses per revolution)
  private static final double k_shoulderDegreesPerPulse = 2.0 * Math.PI / 4096.0;
  private static final double k_elbowDegreesPerPulse = 2.0 * Math.PI / 4096.0;
  private static final double k_wristDegreesPerPulse = 2.0 * Math.PI / 4096.0;

  private final PIDController m_shoulderPID = new PIDController(k_shoulderMotorP, k_shoulderMotorI, k_shoulderMotorD);
  private final PIDController m_elbowPID = new PIDController(k_elbowMotorP, k_elbowMotorI, k_elbowMotorD);
  private final PIDController m_wristPID = new PIDController(k_wristMotorP, k_wristMotorI, k_wristMotorD);

  private final CANSparkMax m_shoulderMotor = new CANSparkMax(Constants.Arm.Shoulder.k_motorId, MotorType.kBrushless);
  private final CANSparkMax m_elbowMotor = new CANSparkMax(Constants.Arm.Elbow.k_motorId, MotorType.kBrushless);
  private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.Arm.Wrist.k_motorId, MotorType.kBrushless);

  private final DoubleSolenoid m_gripper = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
  private boolean m_gripperEngaged = false;

  private final DutyCycleEncoder m_shoulderEncoder = new DutyCycleEncoder(Constants.Arm.Shoulder.k_encoderId);
  private final DutyCycleEncoder m_elbowEncoder = new DutyCycleEncoder(Constants.Arm.Elbow.k_encoderId);
  private final DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.Arm.Wrist.k_encoderId);

  private final RelativeEncoder tempShoulderEncoder = m_shoulderMotor.getEncoder(Type.kHallSensor, 42);
  private final double shoulderStart;

  // private final TrajectoryConfig m_trajConfig = new TrajectoryConfig(k_maxTrajectorySpeed, k_maxTrajectoryAcceleration);
  private ArmTrajectory m_currentTrajectory;
  private boolean m_runningTrajectory = false;
  private Timer m_trajTimer = new Timer();

  private static class PeriodicIO {
    // Automated control
    public double shoulderAngle = 0.0;
    public double elbowAngle = 0.0;
    public double wristAngle = 0.0;

    // Manual control
    public double shoulderMotorPower = 0.0;
    public double elbowMotorPower = 0.0;
    public double wristMotorPower = 0.0;
  }

  private PeriodicIO m_periodicIO = new PeriodicIO();

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
    }
    return m_arm;
  }

  private Arm() {
    m_armSim = ArmSim.getInstance();

    // m_shoulderEncoder.setDistancePerRotation(k_shoulderDegreesPerPulse);
    // m_elbowEncoder.setDistancePerRotation(k_elbowDegreesPerPulse);
    // m_wristEncoder.setDistancePerRotation(k_wristDegreesPerPulse);

    tempShoulderEncoder.setPositionConversionFactor((1.0 / 80.0) * (16.0 / 72.0));
    shoulderStart = tempShoulderEncoder.getPosition();

    // TODO: do this for shoulder and wrist as well
    m_shoulderPID.enableContinuousInput(0, 360);
    m_elbowPID.enableContinuousInput(0, 360);
    m_wristPID.enableContinuousInput(0, 360);

    m_wristMotor.setInverted(true);

    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    System.out.println("Hey, I just met you,\nAnd this is CRAZY\nBut here's my number,\nSo call me, maybe");
  }

  @Override
  public void periodic() {
    //////////////
    // SHOULDER //
    //////////////
    m_periodicIO.shoulderMotorPower = m_shoulderPID.calculate(getShoulderPositionDegrees(), m_periodicIO.shoulderAngle);
    m_shoulderMotor.setVoltage(m_periodicIO.shoulderMotorPower);

    ///////////
    // ELBOW //
    ///////////
    m_periodicIO.elbowMotorPower = m_elbowPID.calculate(getElbowPositionDegrees(), m_periodicIO.elbowAngle);
    m_elbowMotor.setVoltage(m_periodicIO.elbowMotorPower);

    ///////////
    // WRIST //
    ///////////
    // We HAVE to call calculate here, even if we don't use the output, because it's
    // the method that updates the internal position error of the PID controller.
    double wristPIDOutput = m_wristPID.calculate(getWristPositionDegrees(), m_periodicIO.wristAngle);
    if (!m_wristPID.atSetpoint()) {
      m_periodicIO.wristMotorPower = wristPIDOutput;
    } else {
      m_periodicIO.wristMotorPower = 0.0;
    }
    m_periodicIO.wristMotorPower = wristPIDOutput;
    m_wristMotor.setVoltage(m_periodicIO.wristMotorPower);
  }

  public void manual(double shoulder, double elbow, double wrist) {
    m_shoulderMotor.set(shoulder);
    m_elbowMotor.set(elbow);
    m_wristMotor.set(wrist);
  }

  private boolean hasOppositeSigns(double x, double y) {
    return (x < 0 &&  y >= 0) || (x >= 0 && y < 0);
  }

  private ArrayList<ArmPose> getPath(double startX, double startY, double endX, double endY) {
    ArrayList<ArmPose> path = new ArrayList<>();
    path.add(new ArmPose(startX, startY, null));

    if(hasOppositeSigns(startX, endX)) {
      if(startY < Constants.Arm.Preset.HOME.getPose().getY()) {
        path.add(new ArmPose(startX, Constants.Arm.k_homeHeight, null));
      }

      if(startX < 0 && startX < -Constants.Robot.k_length / 2) {
        path.add(new ArmPose(-Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight, null));
      } else if(startX > 0 && startX > Constants.Robot.k_length / 2) {
        path.add(new ArmPose(Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight, null));
      }
      
      path.add(Constants.Arm.Preset.HOME.getPose());

      if(endX < 0 && endX < -Constants.Robot.k_length / 2) {
        path.add(new ArmPose(-Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight, null));
      } else if(endX > 0 && endX > Constants.Robot.k_length / 2) {
        path.add(new ArmPose(Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight, null));
      }

      if(endY < Constants.Arm.k_homeHeight) {
        path.add(new ArmPose(endX, Constants.Arm.k_homeHeight, null));
      }
    }

    path.add(new ArmPose(endX, endY, null));
    return path;
  }

  private boolean isArmPositionValid(double x, double y) {
    // Is real number pass
    if (Double.isNaN(x) || Double.isNaN(y)) {
      return false;
    }

    //Is robot going to die pass
    if(Math.abs(x) < Constants.Robot.k_length / 2 && y < Constants.Arm.k_homeHeight) {
      return false;
    }

    // Is legal position pass
    if (Math.abs(x) > 48 + (Constants.Robot.k_length / 2) || y > 78) {
      return false;
    }

    // Happy ^_^
    return true;
  }

  public double[] setArmPosition(double x, double y) {
    double[] armAngles = calcAngles(x, y);

    if (Double.isNaN(armAngles[0]) || Double.isNaN(armAngles[1])) {
      return armAngles;
    }

    SmartDashboard.putBoolean(m_smartDashboardKey + "PositionIsValid", isArmPositionValid(x, y));

    if (isArmPositionValid(x, y)) {
      m_periodicIO.shoulderAngle = armAngles[0];
      m_periodicIO.elbowAngle = armAngles[1];

      Preferences.setDouble("targetX", x);
      Preferences.setDouble("targetY", y);
      m_armSim.updateArmPosition(armAngles[0], armAngles[1], m_periodicIO.wristAngle, x, y);
    }

    return armAngles;
  }

  /**
   * Given a target pose location of the arm, generate a trajectory
   * @param targetPose (units of inches, and rotation of the wrist)
   */
  public void generateTrajectoryToPose(ArmPose targetPose) {
    double[] currentAngles = m_armSim.getArmAngles();
    double[] currentXY = calcXY(currentAngles[0], currentAngles[1]);

    ArrayList<ArmPose> waypoints = m_arm.getPath(currentXY[0], currentXY[1], targetPose.getX(), targetPose.getY());
    m_currentTrajectory = new ArmTrajectory(waypoints, 10);
  }

  public void startTrajectory() {
    if(m_runningTrajectory == false) {
      m_runningTrajectory = true;
      m_trajTimer.reset();
      m_trajTimer.start();
    }
  }

  public boolean runTrajectory() {
    if(m_runningTrajectory) {
      ArmPose trajPose = m_currentTrajectory.sample(m_trajTimer.get());
      double[] trajAngles = setArmPosition(trajPose.getX(), trajPose.getY());
      // m_periodicIO.shoulderAngle = trajAngles[0];
      // m_periodicIO.elbowAngle = trajAngles[1];

      if(m_currentTrajectory.getTotalTime() < m_trajTimer.get()) {
        m_runningTrajectory = false;
      }
    }
    // else {
    //   double[] currentAngles = m_armSim.getArmAngles();
    //   double[] currentXY = calcXY(currentAngles[0], currentAngles[1]);
    //   Pose2d currentPose = new Pose2d(currentXY[0], currentXY[1], new Rotation2d(0));
    // }

    return m_runningTrajectory;
  }

  public void stopTrajectory() {
    m_runningTrajectory = false;
    m_trajTimer.stop();
  }

  public void rotateWrist() {
    m_periodicIO.wristAngle -= 90;
  }

  public void setWristAngle(double wristAngle) {
    m_periodicIO.wristAngle = wristAngle;
  }

  /**
   * (0, 11.5) is the desired default position :P
   *
   * @param x Horizontal distance from the center of the robot
   * @param y Vertical distance from the floor
   * @return An array containing the shoulder and elbow target angles
   */
  public double[] calcAngles(double x, double y) {
    double L3 = Math.sqrt(Math.pow(x, 2) + Math.pow(y - Constants.Arm.k_shoulderPivotHeight, 2));

    double alpha = Math.acos(
        (Math.pow(Constants.Arm.Shoulder.k_length, 2) + Math.pow(L3, 2) - Math.pow(Constants.Arm.Elbow.k_length, 2))
            / (2 * Constants.Arm.Shoulder.k_length * L3));

    double psi = Math.atan2(x, y - Constants.Arm.k_shoulderPivotHeight);

    double shoulderTargetAngle = x >= 0 ? psi - alpha : psi + alpha;

    // The position of the end of the first arm
    double xPrime = Constants.Arm.Shoulder.k_length * Math.sin(shoulderTargetAngle);
    double yPrime = Constants.Arm.k_shoulderPivotHeight
        + Constants.Arm.Shoulder.k_length * Math.cos(shoulderTargetAngle);

    double elbowTargetAngle = Math.atan2(x - xPrime, yPrime - y);

    shoulderTargetAngle = Units.radiansToDegrees(shoulderTargetAngle);
    elbowTargetAngle = Units.radiansToDegrees(elbowTargetAngle);

    return new double[] { shoulderTargetAngle, elbowTargetAngle };
  }

  /**
   *
   * @param shoulderAngle Shoulder angle (degrees) from vertical (-90 to 90)
   * @param elbowAngle    Elbow angle (degrees) from vertical (-150 to 150)
   * @return An array containing the x, y position in inches
   */
  public double[] calcXY(double shoulderAngle, double elbowAngle) {
    double shoulderAngleRad = Math.toRadians(shoulderAngle);
    double elbowAngleRad = Math.toRadians(elbowAngle);
    // Position at the end of the first arm
    double xPrime = Constants.Arm.Shoulder.k_length * Math.sin(shoulderAngleRad);
    double yPrime = Constants.Arm.k_shoulderPivotHeight
        + Constants.Arm.Shoulder.k_length * Math.cos(shoulderAngleRad);

    double x = xPrime + Constants.Arm.Elbow.k_length * Math.sin(elbowAngleRad);
    double y = yPrime - Constants.Arm.Elbow.k_length * Math.cos(elbowAngleRad);
    return new double[] { x, y };
  }

  public double getShoulderPositionDegrees() {
    // double value = m_shoulderEncoder.getAbsolutePosition() -
    // Constants.Arm.Shoulder.k_offset;
    double value = tempShoulderEncoder.getPosition() - shoulderStart;
    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public double getElbowPositionDegrees() {
    double value = m_elbowEncoder.getAbsolutePosition() - Constants.Arm.Elbow.k_offset;
    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public double getWristPositionDegrees() {
    double value = m_wristEncoder.getAbsolutePosition() - Constants.Arm.Wrist.k_offset;
    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public void setGripper(boolean engaged) {
    m_gripper.set(engaged ? Value.kForward : Value.kReverse);
    m_gripperEngaged = engaged;
  }

  public boolean getGripperEngaged() {
    return m_gripperEngaged;
  }

  public void clearPIDAccumulation() {
    m_shoulderPID.reset();
    m_elbowPID.reset();
    m_wristPID.reset();
  }

  @Override
  public void stop() {
    m_shoulderMotor.set(0);
    m_elbowMotor.set(0);
    m_wristMotor.set(0);
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
    // Shoulder
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Position", getShoulderPositionDegrees());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/PositionError", m_shoulderPID.getPositionError());
    SmartDashboard.putBoolean(m_smartDashboardKey + "Shoulder/AtTarget", m_shoulderPID.atSetpoint());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Velocity", m_shoulderMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Temperature", m_shoulderMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Current", m_shoulderMotor.getOutputCurrent());

    // Elbow
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Position", getElbowPositionDegrees());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Velocity", m_elbowMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Temperature", m_elbowMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Current", m_elbowMotor.getOutputCurrent());

    // Wrist
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Position", getWristPositionDegrees());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/PositionError", m_wristPID.getPositionError());
    SmartDashboard.putBoolean(m_smartDashboardKey + "Wrist/AtTarget", m_wristPID.atSetpoint());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Velocity", m_wristMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Temperature", m_wristMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Current", m_wristMotor.getOutputCurrent());
  }

  public void rezero() {
    tempShoulderEncoder.setPosition(0);
  }
}
