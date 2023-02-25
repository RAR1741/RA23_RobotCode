package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class Arm extends Subsystem {
  private static Arm m_arm = null;

  private static final double k_shoulderMotorP = 1.0;
  private static final double k_shoulderMotorI = 0.0;
  private static final double k_shoulderMotorD = 0.0;

  private static final double k_elbowMotorP = 1.0;
  private static final double k_elbowMotorI = 0.0;
  private static final double k_elbowMotorD = 0.0;

  private static final double k_wristMotorP = 1.0;
  private static final double k_wristMotorI = 0.0;
  private static final double k_wristMotorD = 0.0;

  private static final double k_shoulderGearRatio = 54.0;
  private static final double k_elbowGearRatio = 36.0;
  private static final double k_wristGearRatio = 70.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  private static final double k_shoulderDegreesPerPulse = 2.0 * Math.PI / 4096.0;
  private static final double k_elbowDegreesPerPulse = 2.0 * Math.PI / 4096.0;
  private static final double k_wristDegreesPerPulse = 2.0 * Math.PI / 4096.0;

  private final DCMotor m_shoulderGearbox = DCMotor.getNEO(1);
  private final DCMotor m_elbowGearbox = DCMotor.getNEO(1);
  private final DCMotor m_wristGearbox = DCMotor.getNEO(1);

  private final PIDController m_shoulderPID = new PIDController(k_shoulderMotorP, k_shoulderMotorI, k_shoulderMotorD);
  private final PIDController m_elbowPID = new PIDController(k_elbowMotorP, k_elbowMotorI, k_elbowMotorD);
  private final PIDController m_wristPID = new PIDController(k_wristMotorP, k_wristMotorI, k_wristMotorD);

  private final CANSparkMax m_shoulderMotor = new CANSparkMax(Constants.Arm.Shoulder.k_motorId, MotorType.kBrushless);
  private final CANSparkMax m_elbowMotor = new CANSparkMax(Constants.Arm.Elbow.k_motorId, MotorType.kBrushless);
  private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.Arm.Wrist.k_motorId, MotorType.kBrushless);

  // private final PWMSparkMax m_shoulderMotor = new PWMSparkMax(1); //ew pwm
  // private final PWMSparkMax m_elbowMotor = new PWMSparkMax(2);
  // private final PWMSparkMax m_wristMotor = new PWMSparkMax(3);
  private final SingleJointedArmSim m_shoulderSim = new SingleJointedArmSim(
      m_shoulderGearbox,
      k_shoulderGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Shoulder.k_length),
          Constants.Arm.Shoulder.k_mass), // TODO: include the mass of the 2nd arm
      Units.inchesToMeters(Constants.Arm.Shoulder.k_length),
      Constants.Arm.Shoulder.k_minAngle,
      Constants.Arm.Shoulder.k_maxAngle,
      true);

  private final SingleJointedArmSim m_elbowSim = new SingleJointedArmSim(
      m_elbowGearbox,
      k_elbowGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Elbow.k_length),
          Constants.Arm.Elbow.k_mass),
      Units.inchesToMeters(Constants.Arm.Elbow.k_length),
      Constants.Arm.Elbow.k_minAngle,
      Constants.Arm.Elbow.k_maxAngle,
      true);

  private final SingleJointedArmSim m_wristSim = new SingleJointedArmSim(
      m_wristGearbox,
      k_wristGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Wrist.k_length),
          Constants.Arm.Wrist.k_mass),
      Units.inchesToMeters(Constants.Arm.Wrist.k_length),
      Constants.Arm.Wrist.k_minAngle,
      Constants.Arm.Wrist.k_maxAngle,
      true);

  // This isn't a real encoder...
  // private final Encoder m_shoulderEncoder = new Encoder(20, 21);
  // private final EncoderSim m_shoulderEncoderSim = new EncoderSim(m_shoulderEncoder);

  private final DutyCycleEncoder m_elbowEncoder = new DutyCycleEncoder(Constants.Arm.Elbow.k_encoderId);
  // private final EncoderSim m_elbowEncoderSim = new EncoderSim(m_elbowEncoder);

  private final DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.Arm.Wrist.k_encoderId);
  // private final EncoderSim m_wristEncoderSim = new EncoderSim(m_wristEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmBase and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(Constants.Simulation.k_width, Constants.Simulation.k_height);

  private final MechanismRoot2d m_shoulderPivot = m_mech2d.getRoot("ArmShoulderPivot", Constants.Simulation.k_width / 2,
      Constants.Arm.k_shoulderPivotHeight);

  private final MechanismLigament2d m_armBase = m_shoulderPivot.append(
      new MechanismLigament2d(
          "ArmBase",
          Constants.Arm.k_shoulderPivotHeight,
          -90,
          4,
          new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d m_arm1 = m_shoulderPivot.append(
      new MechanismLigament2d(
          "Arm1",
          Constants.Arm.Shoulder.k_length,
          Units.radiansToDegrees(m_shoulderSim.getAngleRads()),
          4,
          new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d m_arm2 = m_arm1.append(
      new MechanismLigament2d(
          "Arm2",
          Constants.Arm.Elbow.k_length,
          Units.radiansToDegrees(m_elbowSim.getAngleRads()),
          4,
          new Color8Bit(Color.kGreen)));

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
    // m_shoulderEncoder.setDistancePerPulse(k_shoulderDegreesPerPulse);
    m_elbowEncoder.setDistancePerRotation(k_elbowDegreesPerPulse);
    m_wristEncoder.setDistancePerRotation(k_wristDegreesPerPulse);

    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    addAdditionalDrawings();

    SmartDashboard.putData("Arm Sim", m_mech2d);

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

  private void addAdditionalDrawings() {
    // Draw the robot's bumpers
    double bumperPosition = Constants.Simulation.k_width / 2 - Constants.Robot.k_length / 2;
    m_mech2d.getRoot("Robot", bumperPosition, Constants.Robot.k_bumperStart).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Robot.k_length,
            0,
            Constants.Robot.k_bumperHeight,
            new Color8Bit(Color.kRed)));

    // Draw the scoring grid base
    double gridStartPosition = Constants.Simulation.k_width / 2 + Constants.Robot.k_length / 2;
    m_mech2d.getRoot("GridGround", gridStartPosition, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.k_highGoalX,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw the low goal
    m_mech2d.getRoot("GridLowGoal", gridStartPosition + Constants.Field.k_lowGoalX, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.k_lowGoalHeight,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw the high goal
    m_mech2d.getRoot("GridHighGoal", gridStartPosition + Constants.Field.k_highGoalX, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.k_highGoalHeight,
            90,
            5,
            new Color8Bit(Color.kWhite)));
  }

  @Override
  public void periodic() {
  }

  public void manual(double shoulder, double elbow, double wrist) {
    m_shoulderMotor.set(shoulder);
    m_elbowMotor.set(elbow);
    m_wristMotor.set(wrist);
  }

  //TODO I don't really understand this and didn't want to break anything, so I just disabled it
  /*@Override
  public void periodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_shoulderSim.setInput(m_shoulderMotor.get() * RobotController.getBatteryVoltage());
    m_elbowSim.setInput(m_elbowMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_shoulderSim.update(0.020);
    m_elbowSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_shoulderEncoderSim.setDistance(m_shoulderSim.getAngleRads());
    m_elbowEncoderSim.setDistance(m_elbowSim.getAngleRads());

    // SimBattery estimates loaded battery voltages RoboRioSim.setVInVoltage(
    BatterySim.calculateDefaultBatteryLoadedVoltage(
        m_shoulderSim.getCurrentDrawAmps() + m_elbowSim.getCurrentDrawAmps());

    m_periodicIO.shoulderAngle = Preferences.getDouble("shoulderAngle", m_periodicIO.shoulderAngle);
    m_periodicIO.elbowAngle = Preferences.getDouble("elbowAngle", m_periodicIO.elbowAngle);

    double shoulderPIDOutput = m_shoulderPID.calculate(m_shoulderEncoder.getDistance(),
        Units.degreesToRadians(m_periodicIO.shoulderAngle));
    double elbowPIDOutput = m_elbowPID.calculate(m_elbowEncoder.getDistance(),
        Units.degreesToRadians(m_periodicIO.elbowAngle));

    SmartDashboard.putNumber("Shoulder Diff",
        Units.radiansToDegrees(m_shoulderEncoder.getDistance() - Units.degreesToRadians(m_periodicIO.shoulderAngle)));
    SmartDashboard.putNumber("Elbow Diff",
        Units.radiansToDegrees(m_elbowEncoder.getDistance() - Units.degreesToRadians(m_periodicIO.elbowAngle)));

    m_shoulderMotor.setVoltage(shoulderPIDOutput);
    m_elbowMotor.setVoltage(elbowPIDOutput);

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm1.setAngle(Units.radiansToDegrees(m_shoulderSim.getAngleRads()));
    m_arm2.setAngle(Units.radiansToDegrees(m_elbowSim.getAngleRads()));
  }*/
  
  /**
   * 
   * @param x Horizontal distance from the center of the robot
   * @param y Vertical distance from the floor
   * @return An array containing the shoulder and elbow target angles
   */
  public double[] calcAngles(double x, double y) {
    double L3 = Math.sqrt(Math.pow(x, 2) + Math.pow(y - Constants.Arm.k_shoulderPivotHeight, 2));
    
    double alpha = Math.acos((Math.pow(Constants.Arm.Shoulder.k_length, 2) + Math.pow(L3, 2) - Math.pow(Constants.Arm.Elbow.k_length, 2)) 
    / (2 * Constants.Arm.Shoulder.k_length * L3));

    double shoulderTargetAngle = Math.atan2(y - Constants.Arm.k_shoulderPivotHeight, x) - alpha;

    double elbowTargetAngle = Math.asin((L3 * Math.sin(alpha)) / Constants.Arm.Elbow.k_length);

    return new double[] {shoulderTargetAngle, elbowTargetAngle};
  }

  //TODO Add wrist limit
  public void rotWrist(double addedAngle) { //This should work (always needs to be positive)
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

    double shoulderPIDOutput = 0; /*m_shoulderPID.calculate(m_shoulderEncoder.getDistance(), // TODO: Fix this
        Units.degreesToRadians(m_periodicIO.shoulderAngle));*/
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
    SmartDashboard.putNumber("Arm/Wrist Position",m_wristEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm/Elbow/Position",m_elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm/Elbow/Velocity", m_elbowMotor.get());
    SmartDashboard.putNumber("Arm/Elbow/Temperature", m_elbowMotor.getMotorTemperature());
    SmartDashboard.putNumber("Arm/Elbow/Current", m_elbowMotor.getOutputCurrent());
  }
}
