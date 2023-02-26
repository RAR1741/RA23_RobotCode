package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.ArmSim;

public class Arm extends Subsystem {
  private static Arm m_arm = null;
  private static ArmSim m_armSim = null;

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

  private final DutyCycleEncoder m_shoulderEncoder = new DutyCycleEncoder(Constants.Arm.Shoulder.k_encoderId);
  private final DutyCycleEncoder m_elbowEncoder = new DutyCycleEncoder(Constants.Arm.Elbow.k_encoderId);
  private final DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.Arm.Wrist.k_encoderId);

  private static class PeriodicIO {
    // Automated control
    public double shoulderAngle;
    public double elbowAngle;
    public double wristAngle;

    // Manual control
    public double shoulderMotorPower;
    public double elbowMotorPower;
    public double wristMotorPower;
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

    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    System.out.println("Hey, I just met you,\nAnd this is CRAZY\nBut here's my number,\nSo call me, maybe");

    if (!Preferences.containsKey("shoulderAngle")) {
      Preferences.setDouble("shoulderAngle", m_periodicIO.shoulderAngle);
    }
    if (!Preferences.containsKey("elbowAngle")) {
      Preferences.setDouble("elbowAngle", m_periodicIO.elbowAngle);
    }
    if (!Preferences.containsKey("wristAngle")) {
      Preferences.setDouble("wristAngle", m_periodicIO.wristAngle);
    }
  }

  @Override
  public void periodic() {
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

  private void setAngle(double shoulderAngle) {
    setAngle(shoulderAngle, m_periodicIO.elbowAngle, m_periodicIO.wristAngle);
  }

  private void setAngle(double shoulderAngle, double elbowAngle) {
    setAngle(shoulderAngle, elbowAngle, m_periodicIO.wristAngle);
  }

  private void setAngle(double shoulderAngle, double elbowAngle, double wristAngle) {
    m_periodicIO.shoulderAngle = shoulderAngle;
    m_periodicIO.elbowAngle = elbowAngle;
    m_periodicIO.wristAngle = wristAngle;

    double shoulderPIDOutput = 0; /*
                                   * m_shoulderPID.calculate(m_shoulderEncoder.getDistance(), // TODO: Fix this
                                   * Units.degreesToRadians(m_periodicIO.shoulderAngle));
                                   */
    double elbowPIDOutput = m_elbowPID.calculate(m_elbowEncoder.getDistance(),
        Units.degreesToRadians(m_periodicIO.elbowAngle));
    double wristPIDOutput = m_wristPID.calculate(m_wristEncoder.getDistance(),
        Units.degreesToRadians(m_periodicIO.wristAngle));

    // m_shoulderMotor.setVoltage(shoulderPIDOutput);
    // m_elbowMotor.setVoltage(elbowPIDOutput);
    // m_wristMotor.setVoltage(wristPIDOutput);
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
    SmartDashboard.putNumber("Arm/Wrist Position", m_wristEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm/Elbow/Position", m_elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm/Elbow/Velocity", m_elbowMotor.get());
    SmartDashboard.putNumber("Arm/Elbow/Temperature", m_elbowMotor.getMotorTemperature());
    SmartDashboard.putNumber("Arm/Elbow/Current", m_elbowMotor.getOutputCurrent());
  }
}
