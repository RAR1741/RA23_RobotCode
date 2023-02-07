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
  private final String m_smartDashboardKey;
  private final boolean m_DriveMotorInverted;

  private final TalonFXSensorCollection m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

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
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double turningOffset, String moduleName,
      boolean inverted) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;
    m_DriveMotorInverted = inverted;

    m_smartDashboardKey = "Drivetrain/" + m_moduleName + "/";

    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_driveMotor.setInverted(m_DriveMotorInverted);
    m_driveEncoder = m_driveMotor.getSensorCollection();

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_turningMotor.setInverted(true);
    // TODO: maybe we should be doing this too?
    // m_turningMotor.restoreFactoryDefaults();

    // Limit the PID Controller's input range between 0 and 1 and set the input to
    // be continuous.
    m_turningPIDController.enableContinuousInput(0, 1);
    m_turningPIDController.setTolerance(0.05);
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

  public void resetTurnPIDState() {
    m_turningPIDController.reset(getTurnPosition());
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

    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveEncoder.getIntegratedSensorPosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());
  }
}
