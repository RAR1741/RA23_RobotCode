package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final TalonFXSensorCollection m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      1,
      0,
      0,
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
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    // m_turningMotor.restoreFactoryDefaults();

    // Probably need to update these to match the encoder channels from the
    // brushless motors
    m_driveEncoder = m_driveMotor.getSensorCollection();
    // m_turningMotor.get
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // m_turningEncoder.setAverageDepth(12);
    // m_turningEncoder = m_turningMotor.getAlternateEncoder(Type.kQuadrature,
    // kEncoderResolution); //turningMotorChannel); //.getEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // TODO: Figure this out
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // TODO: Figure this out
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getIntegratedSensorVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  public double getTurnPosition() {
    return m_turningEncoder.getPosition();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getIntegratedSensorPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // TODO: Add this back, please (and use it)
    // SwerveModuleState state = SwerveModuleState.optimize(desiredState, new
    // Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    // final double driveOutput =
    // m_drivePIDController.calculate(m_driveEncoder.getIntegratedSensorVelocity(),
    // desiredState.speedMetersPerSecond);

    // final double driveFeedforward =
    // m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
        desiredState.angle.getRadians() / (2 * Math.PI));

    // final double turnFeedforward =
    // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    m_turningMotor.setVoltage(turnOutput);
  }
}
