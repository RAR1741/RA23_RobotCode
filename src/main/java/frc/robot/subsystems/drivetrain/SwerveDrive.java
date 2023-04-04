package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
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
  private final Limelight m_limelight = Limelight.getInstance();

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)) // TODO: CLARIFY THIS WORKS
  );

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
    m_gyro.setAngleAdjustment(0.0);
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void clearTurnPIDAccumulation() {
    m_frontLeft.clearTurnPIDAccumulation();
    m_frontRight.clearTurnPIDAccumulation();
    m_backLeft.clearTurnPIDAccumulation();
    m_backRight.clearTurnPIDAccumulation();
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
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
    m_poseEstimator = new SwerveDrivePoseEstimator(
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
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    );

    setPose(pose);
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

    double maxBoostSpeed = Constants.Drivetrain.k_maxSpeed * Constants.Drivetrain.k_boostScaler;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxBoostSpeed);

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

  public void pointInwards() {
    SwerveModuleState flState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState frState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    SwerveModuleState blState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    SwerveModuleState brState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    m_frontLeft.setDesiredState(flState);
    m_frontRight.setDesiredState(frState);
    m_backLeft.setDesiredState(blState);
    m_backRight.setDesiredState(brState);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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
  }

  private double[] getCurrentStates() {
    double[] currentStates = {
        m_frontLeft.getTurnPosition() * 360, m_frontLeft.getDriveVelocity(),
        m_frontRight.getTurnPosition() * 360, m_frontRight.getDriveVelocity(),
        m_backLeft.getTurnPosition() * 360, m_backLeft.getDriveVelocity(),
        m_backRight.getTurnPosition() * 360, m_backRight.getDriveVelocity()
    };

    return currentStates;
  }

  private double[] getDesiredStates() {
    double[] desiredStates = {
        m_frontLeft.getDesiredState().angle.getDegrees(), m_frontLeft.getDesiredState().speedMetersPerSecond,
        m_frontRight.getDesiredState().angle.getDegrees(), m_frontRight.getDesiredState().speedMetersPerSecond,
        m_backLeft.getDesiredState().angle.getDegrees(), m_backLeft.getDesiredState().speedMetersPerSecond,
        m_backRight.getDesiredState().angle.getDegrees(), m_backRight.getDesiredState().speedMetersPerSecond
    };

    return desiredStates;
  }

  @Override
  public void outputTelemetry() {
    double currentTime = Timer.getFPGATimestamp();
    m_poseEstimator.updateWithTime(
        currentTime,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    
    m_poseEstimator.addVisionMeasurement(m_limelight.getBotpose2D(), currentTime); // TODO: MAKE SURE THIS WORKS

    m_frontLeft.outputTelemetry();
    m_frontRight.outputTelemetry();
    m_backLeft.outputTelemetry();
    m_backRight.outputTelemetry();

    SmartDashboard.putNumberArray("Drivetrain/CurrentStates", getCurrentStates());
    SmartDashboard.putNumberArray("Drivetrain/DesiredStates", getDesiredStates());

    SmartDashboard.putNumber("Drivetrain/Gyro/AngleDegrees", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Gyro/Pitch", m_gyro.getPitch());
    SmartDashboard.putNumberArray("Drivetrain/Pose",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
  }
}
