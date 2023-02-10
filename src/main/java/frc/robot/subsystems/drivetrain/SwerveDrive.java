package frc.robot.subsystems.drivetrain;

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

  private static SwerveDrive swerve = null;

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public static final double kRobotLength = Constants.Drivetrain.kYDistance;
  public static final double kRobotWidth = Constants.Drivetrain.kXDistance;
  public static final double kRobotDiameter = Math.hypot(kRobotLength, kRobotWidth);

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.Drivetrain.kXCenterDistance,
      Constants.Drivetrain.kYCenterDistance);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.Drivetrain.kXCenterDistance,
      -Constants.Drivetrain.kYCenterDistance);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.Drivetrain.kXCenterDistance,
      Constants.Drivetrain.kYCenterDistance);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.Drivetrain.kXCenterDistance,
      -Constants.Drivetrain.kYCenterDistance);

  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.Drivetrain.Drive.kFLDriveMotorId, Constants.Drivetrain.Turn.kFLTurnMotorId,
      Constants.Drivetrain.Turn.kFLTurnOffset, "FL", false);
  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.Drivetrain.Drive.kFRDriveMotorId, Constants.Drivetrain.Turn.kFRTurnMotorId,
      Constants.Drivetrain.Turn.kFRTurnOffset, "FR", true);
  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.Drivetrain.Drive.kBLDriveMotorId, Constants.Drivetrain.Turn.kBLTurnMotorId,
      Constants.Drivetrain.Turn.kBLTurnOffset, "BL", false);
  private final SwerveModule m_backRight = new SwerveModule(
      Constants.Drivetrain.Drive.kBRDriveMotorId, Constants.Drivetrain.Turn.kBRTurnMotorId,
      Constants.Drivetrain.Turn.kBRTurnOffset, "BR", true);

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
    if (swerve == null) {
      swerve = new SwerveDrive();
    }
    return swerve;
  }

  private SwerveDrive() {
    resetGyro();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetKinematics() {
    m_frontLeft.resetTurnPIDState();
    m_frontRight.resetTurnPIDState();
    m_backLeft.resetTurnPIDState();
    m_backRight.resetTurnPIDState();
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  }

  public void reset() {
    resetGyro();
    resetKinematics();
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
    double gyroAngleDeg = m_gyro.getAngle();
    double gyroAngleRad = Math.toRadians(gyroAngleDeg);

    double a,b,c,d;
    double ws1,ws2,ws3,ws4;
    double wa1,wa2,wa3,wa4;

    // Normalize all the speeds to be unitless percentages (0.0 to 1.0)
    double x = xSpeed/kMaxSpeed;
    double y = ySpeed/kMaxSpeed;
    double r = rot/kMaxAngularSpeed;
  
    if(fieldRelative)
    {
      double temp = y * Math.cos(gyroAngleRad) + x * Math.sin(gyroAngleRad);
      x = -y * Math.sin(gyroAngleRad) + x * Math.cos(gyroAngleRad);
      y = temp;
    }

    a = x - r * (kRobotLength/kRobotDiameter);
    b = x + r * (kRobotLength/kRobotDiameter);
    c = y - r * (kRobotWidth/kRobotDiameter);
    d = y + r * (kRobotWidth/kRobotDiameter);

    ws1 = Math.sqrt(Math.pow(b,2) + Math.pow(c,2));
    ws2 = Math.sqrt(Math.pow(b,2) + Math.pow(d,2));
    ws3 = Math.sqrt(Math.pow(a,2) + Math.pow(d,2));
    ws4 = Math.sqrt(Math.pow(a,2) + Math.pow(c,2));
    double max = 0;
    if(ws1 > max){max = ws1;}
    if(ws2 > max){max = ws2;}
    if(ws3 > max){max = ws3;}
    if(ws4 > max){max = ws4;}
    if(max > 1){ws1 /= max;ws2 /= max;ws3 /= max;ws4 /= max;}

    wa1 = Math.toDegrees(Math.atan2(b,c));
    wa2 = Math.toDegrees(Math.atan2(b,d));
    wa3 = Math.toDegrees(Math.atan2(a,d));
    wa4 = Math.toDegrees(Math.atan2(a,c));
    if(wa1 < 0){wa1 += 360;}//wa1 = FL
    if(wa2 < 0){wa2 += 360;}//wa2 = FR
    if(wa3 < 0){wa3 += 360;}//wa3 = BR
    if(wa4 < 0){wa4 += 360;}//wa4 = BL
    
    SwerveModuleState flState = new SwerveModuleState(-ws1,Rotation2d.fromDegrees(-wa1));
    SwerveModuleState frState = new SwerveModuleState( ws2,Rotation2d.fromDegrees( wa2));
    SwerveModuleState brState = new SwerveModuleState( ws3,Rotation2d.fromDegrees( wa3));
    SwerveModuleState blState = new SwerveModuleState(-ws4,Rotation2d.fromDegrees(-wa4));

    m_frontLeft.setDesiredState(flState);
    m_frontRight.setDesiredState(frState);
    m_backLeft.setDesiredState(blState);
    m_backRight.setDesiredState(brState);
  }

  public void pointDirection(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Zero out the speed component of each swerve module state
    for(SwerveModuleState moduleState : swerveModuleStates) {
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
