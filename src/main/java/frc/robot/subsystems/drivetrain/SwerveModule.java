package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double k_wheelRadiusIn = 2; // 2 inches
  private static final double k_driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double k_driveEncPerRot = 2048.0;
  private static final double k_driveEncPerSec = 2048.0 / 10.0; // Encoder reports 2048 Encoder counter per 100 ms

  private static final double k_turningP = 8.0;
  private static final double k_turningI = 0.1;
  private static final double k_turningD = 0.0;
  private static final double k_turnFeedForwardS = 1;
  private static final double k_turnFeedForwardV = 0.5;
  private static final double k_turnFeedForwardA = 0.0;

  // These values were obtained via SysId
  private static final double k_driveP = 0.80566;
  private static final double k_driveI = 0.0;
  private static final double k_driveD = 0.0;
  private static final double k_driveFeedForwardS = 0.19882;
  private static final double k_driveFeedForwardV = 2.21080;
  private static final double k_driveFeedForwardA = 0.11641;

  // TODO: Make sure these are right
  private static final double k_moduleMaxAngularVelocity = Math.PI;
  private static final double k_moduleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final double m_turningOffset;
  private final String m_moduleName;
  private final String m_smartDashboardKey;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final TalonFXSensorCollection m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(k_driveP, k_driveI, k_driveD);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      k_turningP,
      k_turningI,
      k_turningD,
      new TrapezoidProfile.Constraints(
          k_moduleMaxAngularVelocity, k_moduleMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
      k_driveFeedForwardS,
      k_driveFeedForwardV,
      k_driveFeedForwardA);

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(k_turnFeedForwardS,
      k_turnFeedForwardV, k_turnFeedForwardA);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   CAN output for the drive motor.
   * @param turningMotorChannel CAN output for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double turningOffset, String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_smartDashboardKey = "Drivetrain/" + m_moduleName + "/";

    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_driveEncoder = m_driveMotor.getSensorCollection();

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_turningMotor.setInverted(true);

    // Limit the PID Controller's input range between 0 and 1 and set the input to
    // be continuous.
    m_turningPIDController.enableContinuousInput(0, 1);
  }

  private static class PeriodicIO {
    double turnMotorVoltage = 0.0;
    double driveMotorVoltage = 0.0;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRotations(getTurnPosition()));
  }

  public double getTurnPosition() {
    return modRotations(m_turningEncoder.getPosition() - m_turningOffset);
  }

  public double modRotations(double input) {
    input %= 1.0;
    if (input < 0.0) {
      input += 1.0;
    }
    return input;
  }

  // Returns the drive velocity in meters per second.
  public double getDriveVelocity() {
    // In revs per second
    double velocity = m_driveEncoder.getIntegratedSensorVelocity() / k_driveEncPerSec;

    // Convert to in per second
    velocity *= ((2 * k_wheelRadiusIn * Math.PI) / k_driveGearRatio);

    // Convert to m per second
    velocity *= 0.0254;

    return velocity;
  }

  /**
   * Returns the current position of the module (Meters, Angle).
   *
   * @return The current position of the module (Meters, Angle).
   */
  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveEncoder.getIntegratedSensorPosition();
    drivePosition /= k_driveEncPerRot; // Convert to # of rotations
    drivePosition *= ((2 * k_wheelRadiusIn * Math.PI) / k_driveGearRatio); // Convert to inches
    drivePosition *= 0.0254; // Convert to meters

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRotations(getTurnPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(getTurnPosition()));

    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);

    double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnTarget = desiredState.angle.getRotations();

    double turnOutput = m_turningPIDController.calculate(getTurnPosition(), turnTarget);

    double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    boolean turnAtGoal = m_turningPIDController.atGoal();

    SmartDashboard.putNumber(m_smartDashboardKey + "TurnTarget", turnTarget);
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnOutput", turnOutput + turnFeedforward);
    SmartDashboard.putBoolean(m_smartDashboardKey + "TurnAtGoal", turnAtGoal);
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveTargetVelocity", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveOutput", driveOutput + driveFeedforward);

    m_periodicIO.turnMotorVoltage = turnOutput + turnFeedforward;
    m_periodicIO.driveMotorVoltage = driveOutput + driveFeedforward;
  }

  public void periodic() {
    m_turningMotor.setVoltage(m_periodicIO.turnMotorVoltage);
    m_driveMotor.setVoltage(m_periodicIO.driveMotorVoltage);
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveEncoder.getIntegratedSensorPosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());
  }
}