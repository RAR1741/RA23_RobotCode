package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.ArmSim;

public class Arm extends Subsystem {
  private static Arm m_arm = null;
  private static ArmSim m_armSim = null;

  private final String m_smartDashboardKey = "Arm/";

  private static final double k_shoulderMotorP = 1.0;
  private static final double k_shoulderMotorI = 0.0;
  private static final double k_shoulderMotorD = 0.0;

  private static final double k_elbowMotorP = 1.0;
  private static final double k_elbowMotorI = 0.0;
  private static final double k_elbowMotorD = 0.0;

  private static final double k_wristMotorP = 1.0;
  private static final double k_wristMotorI = 0.0;
  private static final double k_wristMotorD = 0.0;

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

    m_shoulderEncoder.setDistancePerRotation(k_shoulderDegreesPerPulse);
    m_elbowEncoder.setDistancePerRotation(k_elbowDegreesPerPulse);
    m_wristEncoder.setDistancePerRotation(k_wristDegreesPerPulse);

    m_shoulderEncoder.setPositionOffset(Constants.Arm.Shoulder.k_offset);
    m_elbowEncoder.setPositionOffset(Constants.Arm.Elbow.k_offset);
    m_wristEncoder.setPositionOffset(Constants.Arm.Wrist.k_offset);

    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    System.out.println("Hey, I just met you,\nAnd this is CRAZY\nBut here's my number,\nSo call me, maybe");
  }

  @Override
  public void periodic() {
    // m_shoulderMotor.setVoltage(m_periodicIO.shoulderMotorPower);
    // m_elbowMotor.setVoltage(m_periodicIO.elbowMotorPower);
    // m_wristMotor.setVoltage(m_periodicIO.wristMotorPower);
  }

  public void manual(double shoulder, double elbow, double wrist) {
    m_shoulderMotor.set(shoulder);
    m_elbowMotor.set(elbow);
    m_wristMotor.set(wrist);
  }

  public double[] setArmPosition(double x, double y, double wristAngle) {
    double[] armAngles = calcAngles(x, y);

    m_periodicIO.shoulderAngle = armAngles[0];
    m_periodicIO.elbowAngle = armAngles[1];
    // TODO: Do wrist things here too

    m_armSim.updateArmPosition(armAngles[0], armAngles[1], wristAngle, x, y);

    // double shoulderPIDOutput =
    // m_shoulderPID.calculate(m_shoulderEncoder.getDistance(),
    // Units.degreesToRadians(m_periodicIO.elbowAngle));

    double elbowEncoderDegrees = m_elbowEncoder.getDistance() * k_elbowDegreesPerPulse;
    double elbowPIDOutput = m_elbowPID.calculate(elbowEncoderDegrees, m_periodicIO.elbowAngle);

    // double wristPIDOutput = m_wristPID.calculate(m_wristEncoder.getDistance(),
    // Units.degreesToRadians(m_periodicIO.wristAngle));

    // m_periodicIO.shoulderMotorPower = shoulderPIDOutput;
    m_periodicIO.elbowMotorPower = elbowPIDOutput;
    // m_periodicIO.wristMotorPower = wristPIDOutput;

    return armAngles;
  }

  /**
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

    double elbowTargetAngle = Math.asin((x - (Constants.Arm.Shoulder.k_length * Math.sin(shoulderTargetAngle))) /
        Constants.Arm.Elbow.k_length);

    elbowTargetAngle += shoulderTargetAngle;

    shoulderTargetAngle = Units.radiansToDegrees(shoulderTargetAngle);
    elbowTargetAngle = Units.radiansToDegrees(elbowTargetAngle);

    return new double[] { shoulderTargetAngle, elbowTargetAngle };
  }

  // TODO Add wrist limit
  public void rotWrist(double addedAngle) { // This should work (always needs to be positive)
    m_periodicIO.wristAngle += addedAngle;

    double wristPIDOutput = Math.abs(m_wristPID.calculate(m_wristEncoder.getDistance(),
        Units.degreesToRadians(m_periodicIO.wristAngle)));

    m_wristMotor.setVoltage(wristPIDOutput);
  }

  public void setGripper(boolean engaged) {
    m_gripper.set(engaged ? Value.kForward : Value.kReverse);
    m_gripperEngaged = engaged;
  }

  public boolean getGripperEngaged() {
    return m_gripperEngaged;
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
    SmartDashboard.putNumber(m_smartDashboardKey + "Shoulder/Position", m_shoulderEncoder.getAbsolutePosition());

    // Elbow
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Position", m_elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Velocity", m_elbowMotor.get());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Temperature", m_elbowMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_smartDashboardKey + "Elbow/Current", m_elbowMotor.getOutputCurrent());

    // Wrist
    SmartDashboard.putNumber(m_smartDashboardKey + "Wrist/Position", m_wristEncoder.getAbsolutePosition());
  }
}
