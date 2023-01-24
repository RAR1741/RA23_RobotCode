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
  private static final double kWheelRadiusIn = 2; // 2in
  private static final double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double kDriveEncPerSec = 204.8;
  private static final double kEncoderResolution = 4096.0;

  private static final double kTurningP = 8.0;
  private static final double kTurningI = 0.1;
  private static final double kTurningD = 0.0;

  private static final double kDriveP = 1.0;
  private static final double kDriveI = 0.0;
  private static final double kDriveD = 0.0;

  // TODO: Make sure these are right
  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final double m_turningOffset;
  private final String m_moduleName;

  private final TalonFXSensorCollection m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  private final PIDController m_drivePIDController = new PIDController(kDriveP, kDriveI, kDriveD);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      kTurningP,
      kTurningI,
      kTurningD,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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

    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_driveEncoder = m_driveMotor.getSensorCollection();

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_turningMotor.setInverted(true);
    // TODO: maybe we should be doing this too?
    // m_turningMotor.restoreFactoryDefaults();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // TODO: Figure this out
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusIn /
    // kEncoderResolution);

    // Limit the PID Controller's input range between 0 and 1 and set the input to
    // be continuous.
    m_turningPIDController.enableContinuousInput(0, 1);
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
    double tempPos = m_turningEncoder.getPosition() - m_turningOffset;
    if (tempPos < 0) {
      tempPos += 1;
    }
    return tempPos;
  }

  // Returns the drive velocity in meters per second.
  public double getDriveVelocity() {
    // In revs per second
    double velocity = m_driveEncoder.getIntegratedSensorVelocity() / kDriveEncPerSec;

    // Convert to in per second
    velocity *= ((2 * kWheelRadiusIn * Math.PI) / kDriveGearRatio);

    // Convert to m per second
    velocity *= 0.0254;

    return velocity;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getIntegratedSensorPosition(), Rotation2d.fromRotations(getTurnPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // TODO: Add this back, please (and use it)
    // TODO This no worky with offset
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(getTurnPosition()));

    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);

    double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnTarget = desiredState.angle.getRotations();

    double turnOutput = m_turningPIDController.calculate(getTurnPosition(), turnTarget);

    double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    SmartDashboard.putNumber(m_moduleName + ": turnTarget", turnTarget);
    SmartDashboard.putNumber(m_moduleName + ": turnOutput", turnOutput + turnFeedforward);

    SmartDashboard.putNumber(m_moduleName + ": drivePos", m_driveEncoder.getIntegratedSensorPosition());
    SmartDashboard.putNumber(m_moduleName + ": driveVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_moduleName + ": driveTargetVelocity", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName + ": driveOutput", driveOutput + driveFeedforward);

    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
  }
}
