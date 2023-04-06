package frc.robot.subsystems.arm;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

  // Tippy values
  // private static final double k_shoulderMotorP = 0.4;
  // private static final double k_shoulderMotorI = 0.0;
  // private static final double k_shoulderMotorD = 0.0;

  // private static final double k_elbowMotorP = 0.3;
  // private static final double k_elbowMotorI = 0.0;
  // private static final double k_elbowMotorD = 0.0;

  // New test PID values
  private static final double k_shoulderMotorP = 0.250000;
  private static final double k_shoulderMotorI = 0.010000;
  private static final double k_shoulderMotorD = 0.007500;

  private static final double k_elbowMotorP = 0.150000;
  private static final double k_elbowMotorI = 0.007500;
  private static final double k_elbowMotorD = 0.010000;

  private static final double k_wristMotorP = 0.045;
  private static final double k_wristMotorI = 0.003;
  private static final double k_wristMotorD = 0.001;

  private static final double k_armSafetyLengthOffset = 35.0; // inches
  private static final double k_armSafetyHeightOffset = 20.0; // inches
  private static final double k_armSafetyOffsetThreshold = 35; // inches

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

  private ArmTrajectory m_currentTrajectory;
  private boolean m_runningTrajectory = false;
  private boolean m_runningThrow = false;
  private Timer m_trajTimer = new Timer();
  private Timer m_throwTimer = new Timer();

  private boolean m_inverted = false;
  private double m_invertedLengthBoostFactor = 2.0; // inches

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
  private double m_xPosition = 0;
  private double m_yPosition = Constants.Arm.k_homeHeight;
  private boolean m_elbowAntiBoost;

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
    }
    return m_arm;
  }

  private Arm() {
    m_armSim = ArmSim.getInstance();

    // TODO: do this for shoulder and wrist as well
    m_shoulderPID.enableContinuousInput(0, 360);
    m_elbowPID.enableContinuousInput(0, 360);
    m_wristPID.enableContinuousInput(0, 360);

    m_wristMotor.setInverted(true);
    m_elbowMotor.setInverted(false);

    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    System.out.println("Hey, I just met you,\nAnd this is CRAZY\nBut here's my number,\nSo call me, maybe");

    if (!Preferences.containsKey("k_shoulderMotorP")) {
      Preferences.setDouble("k_shoulderMotorP", k_shoulderMotorP);
    }
    if (!Preferences.containsKey("k_shoulderMotorI")) {
      Preferences.setDouble("k_shoulderMotorI", k_shoulderMotorI);
    }
    if (!Preferences.containsKey("k_shoulderMotorD")) {
      Preferences.setDouble("k_shoulderMotorD", k_shoulderMotorD);
    }
    if (!Preferences.containsKey("k_elbowMotorP")) {
      Preferences.setDouble("k_elbowMotorP", k_elbowMotorP);
    }
    if (!Preferences.containsKey("k_elbowMotorI")) {
      Preferences.setDouble("k_elbowMotorI", k_elbowMotorI);
    }
    if (!Preferences.containsKey("k_elbowMotorD")) {
      Preferences.setDouble("k_elbowMotorD", k_elbowMotorD);
    }
  }

  @Override
  public void periodic() {
    // TODO: Comment this out when not tuning PID
    m_shoulderPID.setPID(
        Preferences.getDouble("k_shoulderMotorP", k_shoulderMotorP),
        Preferences.getDouble("k_shoulderMotorI", k_shoulderMotorI),
        Preferences.getDouble("k_shoulderMotorD", k_shoulderMotorD));
    m_elbowPID.setPID(
        Preferences.getDouble("k_elbowMotorP", k_elbowMotorP),
        Preferences.getDouble("k_elbowMotorI", k_elbowMotorI),
        Preferences.getDouble("k_elbowMotorD", k_elbowMotorD));

    //////////////
    // SHOULDER //
    //////////////
    m_periodicIO.shoulderMotorPower = m_shoulderPID.calculate(getShoulderPositionDegrees(), m_periodicIO.shoulderAngle);
    m_shoulderMotor.setVoltage(m_periodicIO.shoulderMotorPower);

    ///////////
    // ELBOW //
    ///////////
    if (m_runningThrow) {
      if (m_throwTimer.get() > 0.010) {
        setGripper(false);
      }

      if (m_throwTimer.get() > 1.0) {
        m_runningThrow = false;
        m_periodicIO.elbowAngle = 0.0;
        m_throwTimer.stop();
        m_throwTimer.reset();
      }
    }

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

  private ArrayList<ArmPose> getPath(double startX, double startY, double endX, double endY) {
    ArrayList<ArmPose> path = new ArrayList<>();
    path.add(new ArmPose(startX, startY, null));

    boolean enteringFront = (startX > Constants.Robot.k_length / 2 && endX < Constants.Robot.k_length / 2);
    boolean enteringBack = (startX < -Constants.Robot.k_length / 2 && endX > -Constants.Robot.k_length / 2);
    boolean exitingFront = (startX < Constants.Robot.k_length / 2 && endX > Constants.Robot.k_length / 2);
    boolean exitingBack = (startX > -Constants.Robot.k_length / 2 && endX < -Constants.Robot.k_length / 2);

    boolean startingLow = (startY < Constants.Arm.k_homeHeight);
    boolean endingLow = (endY < Constants.Arm.k_homeHeight);

    if (startingLow) {
      path.add(new ArmPose(startX, Constants.Arm.k_homeHeight + 3.0639, null));
    }

    // So we don't get caught in the high cube goal
    if (startY >= k_armSafetyOffsetThreshold && Math.abs(startX) > 50) {
      path.add(new ArmPose(startX > 0 ? startX - k_armSafetyLengthOffset : startX + k_armSafetyLengthOffset,
          startY + k_armSafetyHeightOffset, null));
    }

    double xPosThresh = Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose().getX() + 5;
    double xNegThresh = Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose().getX() - 5;
    double yPosThresh = Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose().getY() + 5;
    double yNegThresh = Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose().getY() - 5;

    // Safety point for feeder station -> home
    if (Math.abs(startX) < xPosThresh && Math.abs(startX) > xNegThresh && startY < yPosThresh && startY > yNegThresh) {
      path.add(new ArmPose(startX > 0 ? Constants.Robot.k_length / 2 + 0.1 : -Constants.Robot.k_length / 2 - 0.1,
          startY, null));
    }

    if (enteringFront) {
      path.add(new ArmPose(Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight + 3.0639, null));
      if (exitingBack) {
        path.add(Constants.Arm.Preset.HOME.getPose());
        path.add(new ArmPose(-Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight + 3.0639, null));
      }
    } else if (enteringBack) {
      path.add(new ArmPose(-Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight + 3.0639, null));
      if (exitingFront) {
        path.add(Constants.Arm.Preset.HOME.getPose());
        path.add(new ArmPose(Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight + 3.0639, null));
      }
    } else if (exitingFront) {
      path.add(new ArmPose(Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight + 3.0639, null));
    } else if (exitingBack) {
      path.add(new ArmPose(-Constants.Robot.k_length / 2, Constants.Arm.k_homeHeight + 3.0639, null));
    }

    if (endingLow) {
      path.add(new ArmPose(endX, Constants.Arm.k_homeHeight + 3.0639, null));
    }

    // Safety point for feeder station <- home
    if (Math.abs(endX) < xPosThresh && Math.abs(endX) > xNegThresh && endY < yPosThresh && endY > yNegThresh) {
      path.add(
          new ArmPose(endX > 0 ? Constants.Robot.k_length / 2 + 0.1 : -Constants.Robot.k_length / 2 - 0.1, endY, null));
    }

    path.add(new ArmPose(endX, endY, null));
    return path;
  }

  private boolean isArmPositionValid(double x, double y) {
    // Is real number pass
    if (Double.isNaN(x) || Double.isNaN(y)) {
      return false;
    }

    // Is robot going to die pass
    if (Math.abs(x) < Constants.Robot.k_length / 2 && y < Constants.Arm.k_homeHeight) {
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
      m_periodicIO.elbowAngle = -armAngles[1]; // TODO: elbow angle needs to be reversed

      if (m_elbowAntiBoost) {
        if (m_inverted) {
          m_periodicIO.elbowAngle += 10.0;
        } else {
          m_periodicIO.elbowAngle -= 10.0;
        }
      }

      m_xPosition = x;
      m_yPosition = y;
    }
    m_armSim.updateArmPosition(armAngles[0], armAngles[1], m_periodicIO.wristAngle, x, y);

    return armAngles;
  }

  public void setAntiBoost(boolean value) {
    m_elbowAntiBoost = value;
  }

  /**
   * Given a target pose location of the arm, generate a trajectory
   *
   * @param targetPose (units of inches, and rotation of the wrist)
   */
  public void generateTrajectoryToPose(ArmPose targetPose) {
    double[] currentAngles = m_armSim.getArmAngles();
    double[] currentXY = calcXY(currentAngles[0], currentAngles[1]);

    // double targetX = m_inverted ? -targetPose.getX() -
    // m_invertedLengthBoostFactor : targetPose.getX();
    double targetX = m_inverted ? -targetPose.getX() : targetPose.getX();
    if (targetPose.getX() == Constants.Arm.Preset.HOME.getPose().getX()) {
      targetX = Constants.Arm.Preset.HOME.getPose().getX();
    }

    // Boost the double feeder station height, if inverted
    double targetY = targetPose.getY();
    if (m_inverted && targetPose.getY() == Constants.Arm.Preset.DOUBLE_SUBSTATION.getPose().getY()) {
      targetY += 5.0;
    }
    ArrayList<ArmPose> waypoints = m_arm.getPath(currentXY[0], currentXY[1], targetX, targetY);

    m_currentTrajectory = new ArmTrajectory(waypoints);

    m_arm.startTrajectory();
  }

  public void startTrajectory() {
    if (m_runningTrajectory == false) {
      m_runningTrajectory = true;
      m_trajTimer.reset();
      m_trajTimer.start();
    }
  }

  public void throwCube() {
    if (!m_runningTrajectory && !m_runningThrow && m_periodicIO.shoulderAngle == 0.0) {
      m_runningThrow = true;
      m_throwTimer.reset();
      m_throwTimer.start();
      m_periodicIO.elbowAngle = 160.0;
    }
  }

  public boolean runTrajectory() {
    if (m_runningTrajectory) {
      ArmPose trajPose = m_currentTrajectory.sample(m_trajTimer.get());
      double[] trajAngles = setArmPosition(trajPose.getX(), trajPose.getY());

      if (m_currentTrajectory.getTotalTime() < m_trajTimer.get()) {
        stopTrajectory();
      }
    }

    return m_runningTrajectory;
  }

  public void stopTrajectory() {
    m_runningTrajectory = false;
    m_trajTimer.stop();
  }

  public void rotateWrist() {
    m_periodicIO.wristAngle -= 90;
  }

  public void rotateWrist(double change) {
    m_periodicIO.wristAngle += change;
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
    double shoulderTargetAngle = 0.0;
    double elbowTargetAngle = 0.0;

    double xMod = Math.abs(x);

    // If the x,y position is within the width of the robot, fix the shoulder at 0
    // degrees and only move the elbow
    if (xMod <= Constants.Robot.k_length / 2) {
      // When inside the frame perimeter, we will disregard the y setpoint so that the
      // shoulder can stay fixed at 0
      shoulderTargetAngle = 0.0;

      elbowTargetAngle = Math.asin(x / Constants.Arm.Elbow.k_length);
    } else {
      double L3 = Math.sqrt(Math.pow(xMod, 2) + Math.pow(y - Constants.Arm.k_shoulderPivotHeight, 2));

      double alpha = Math.acos(
          (Math.pow(Constants.Arm.Shoulder.k_length, 2) + Math.pow(L3, 2) - Math.pow(Constants.Arm.Elbow.k_length, 2))
              / (2 * Constants.Arm.Shoulder.k_length * L3));

      double psi = Math.atan2(xMod, Constants.Arm.k_shoulderPivotHeight - y);

      shoulderTargetAngle = Math.PI - alpha - psi;

      // The position of the end of the first arm
      double xPrime = Constants.Arm.Shoulder.k_length * Math.sin(shoulderTargetAngle);
      double yPrime = Constants.Arm.k_shoulderPivotHeight
          + Constants.Arm.Shoulder.k_length * Math.cos(shoulderTargetAngle);

      elbowTargetAngle = Math.acos((yPrime - y) / Constants.Arm.Elbow.k_length) + shoulderTargetAngle;

      if (x < 0) {
        shoulderTargetAngle *= -1;
        elbowTargetAngle *= -1;
      }
    }

    shoulderTargetAngle = Units.radiansToDegrees(shoulderTargetAngle);
    elbowTargetAngle = Units.radiansToDegrees(elbowTargetAngle);

    SmartDashboard.putNumberArray("CalculatedAngles", new double[] { shoulderTargetAngle, elbowTargetAngle });

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
    double value = m_shoulderEncoder.getAbsolutePosition() -
        Constants.Arm.Shoulder.k_offset;
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

  public void adjustPosition(double xChange, double yChange) {
    setArmPosition(m_xPosition + xChange, m_yPosition + yChange);
  }

  public void setInverted(boolean inverted) {
    if (m_xPosition == Constants.Arm.Preset.HOME.getPose().getX()) {
      m_inverted = inverted;
    }
  }

  public boolean getInverted() {
    return m_inverted;
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
    SmartDashboard.putNumber(m_smartDashboardKey + "X", m_xPosition);
    SmartDashboard.putNumber(m_smartDashboardKey + "Y", m_yPosition);
    SmartDashboard.putBoolean(m_smartDashboardKey + "RunningTrajectory", m_runningTrajectory);

    // Shoulder
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Position", getShoulderPositionDegrees());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/AbsPosition", m_shoulderEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/PositionError", m_shoulderPID.getPositionError());
    SmartDashboard.putBoolean(m_smartDashboardKey + "Shoulder/AtTarget", m_shoulderPID.atSetpoint());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Velocity", m_shoulderMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Temperature", m_shoulderMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Current", m_shoulderMotor.getOutputCurrent());

    // Elbow
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Position", getElbowPositionDegrees());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/AbsPosition", m_elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/PositionError", m_elbowPID.getPositionError());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Velocity", m_elbowMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Temperature", m_elbowMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Current", m_elbowMotor.getOutputCurrent());

    // Wrist
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Position", getWristPositionDegrees());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/AbsPosition", m_wristEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/PositionError", m_wristPID.getPositionError());
    SmartDashboard.putBoolean(m_smartDashboardKey + "Wrist/AtTarget", m_wristPID.atSetpoint());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Velocity", m_wristMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Temperature", m_wristMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Current", m_wristMotor.getOutputCurrent());
  }
}
