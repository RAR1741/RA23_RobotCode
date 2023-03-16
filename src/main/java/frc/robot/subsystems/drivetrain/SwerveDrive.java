package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class SwerveDrive extends Subsystem {

  private static SwerveDrive m_swerve = null;

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.Drivetrain.k_xCenterDistance,
      Constants.Drivetrain.k_yCenterDistance);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.Drivetrain.k_xCenterDistance,
      -Constants.Drivetrain.k_yCenterDistance);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.Drivetrain.k_xCenterDistance,
      Constants.Drivetrain.k_yCenterDistance);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.Drivetrain.k_xCenterDistance,
      -Constants.Drivetrain.k_yCenterDistance);

  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.Drivetrain.Drive.k_FLMotorId, Constants.Drivetrain.Turn.k_FLMotorId,
      Constants.Drivetrain.Turn.k_FLOffset, "FL");
  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.Drivetrain.Drive.k_FRMotorId, Constants.Drivetrain.Turn.k_FRMotorId,
      Constants.Drivetrain.Turn.k_FROffset, "FR");
  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.Drivetrain.Drive.k_BLMotorId, Constants.Drivetrain.Turn.k_BLMotorId,
      Constants.Drivetrain.Turn.k_BLOffset, "BL");
  private final SwerveModule m_backRight = new SwerveModule(
      Constants.Drivetrain.Drive.k_BRMotorId, Constants.Drivetrain.Turn.k_BRMotorId,
      Constants.Drivetrain.Turn.k_BROffset, "BR");

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  public static SwerveDrive getInstance() {
    if (m_swerve == null) {
      m_swerve = new SwerveDrive();
    }
    return m_swerve;
  }

  private SwerveDrive() {
    brakeOff();
    reset();
  }

  public void brakeOn() {
    m_frontLeft.getDriveMotor().setNeutralMode(NeutralMode.Brake);
    m_frontRight.getDriveMotor().setNeutralMode(NeutralMode.Brake);
    m_backLeft.getDriveMotor().setNeutralMode(NeutralMode.Brake);
    m_backRight.getDriveMotor().setNeutralMode(NeutralMode.Brake);
  }

  public void brakeOff() {
    m_frontLeft.getDriveMotor().setNeutralMode(NeutralMode.Coast);
    m_frontRight.getDriveMotor().setNeutralMode(NeutralMode.Coast);
    m_backLeft.getDriveMotor().setNeutralMode(NeutralMode.Coast);
    m_backRight.getDriveMotor().setNeutralMode(NeutralMode.Coast);
  }

  public void reset() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Calls the NavX reset function, resetting the Z angle to 0
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }

  public void resetOdometry(Pose2d pose) {
    m_frontLeft.resetDriveEncoder();
    m_frontRight.resetDriveEncoder();
    m_backLeft.resetDriveEncoder();
    m_backRight.resetDriveEncoder();

    // We're manually setting the drive encoder positions to 0, since we
    // just reset them, but the encoder isn't reporting 0 yet.
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_frontLeft.getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_frontRight.getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_backLeft.getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_backRight.getTurnPosition())),
        });

    // setPose(pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drivetrain.k_maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void pointModules(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Zero out the speed component of each swerve module state
    for (SwerveModuleState moduleState : swerveModuleStates) {
      moduleState.speedMetersPerSecond = 0.0;
    }

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();
  }

  @Override
  public void stop() {
    brakeOn();
    drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
  }

  @Override
  public void outputTelemetry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    m_frontLeft.outputTelemetry();
    m_frontRight.outputTelemetry();
    m_backLeft.outputTelemetry();
    m_backRight.outputTelemetry();

    SmartDashboard.putNumber("Drivetrain/Gyro/AngleDegrees", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Gyro/Pitch", m_gyro.getPitch());
    SmartDashboard.putNumberArray("Drivetrain/Pose",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
  }
}
