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

  private static SwerveDrive swerve = null;

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.Drivetrain.kXDistance,
      Constants.Drivetrain.kYDistance);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.Drivetrain.kXDistance,
      -Constants.Drivetrain.kYDistance);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.Drivetrain.kXDistance,
      Constants.Drivetrain.kYDistance);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.Drivetrain.kXDistance,
      -Constants.Drivetrain.kYDistance);

  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.Drivetrain.Drive.kFLDriveMotorId, Constants.Drivetrain.Turn.kFLTurnMotorId,
      Constants.Drivetrain.Turn.kFLTurnOffset, "FL", true);
  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.Drivetrain.Drive.kFRDriveMotorId, Constants.Drivetrain.Turn.kFRTurnMotorId,
      Constants.Drivetrain.Turn.kFRTurnOffset, "FR", false);
  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.Drivetrain.Drive.kBLDriveMotorId, Constants.Drivetrain.Turn.kBLTurnMotorId,
      Constants.Drivetrain.Turn.kBLTurnOffset, "BL", true);
  private final SwerveModule m_backRight = new SwerveModule(
      Constants.Drivetrain.Drive.kBRDriveMotorId, Constants.Drivetrain.Turn.kBRTurnMotorId,
      Constants.Drivetrain.Turn.kBRTurnOffset, "BR", false);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
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
    if (swerve == null) {
      swerve = new SwerveDrive();
    }
    return swerve;
  }

  private SwerveDrive() {
    // TODO: Make a separate method for this
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
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the position of the center of the robot
   * 
   * @return The center position of the robot in meters
   */
  public double getEncoder() {
    return (m_frontLeft.getPosition().distanceMeters + m_backRight.getPosition().distanceMeters) / 2; 
  }

  public AHRS getGyro() {
    return m_gyro;
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
