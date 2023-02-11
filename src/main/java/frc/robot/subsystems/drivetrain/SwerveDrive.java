package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
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

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
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
    resetGyro();
  }

  public void resetGyro() {
    m_gyro.reset();
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
    // WIP: rotation locking
    // rot = -m_gyro.getRotation2d().getRadians() / 2;

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

  public void pointDirection(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
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
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
  }

  @Override
  public void outputTelemetry() {
    // TODO: Add this back in
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
  }
}
