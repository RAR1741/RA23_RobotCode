package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Drivetrain extends Subsystem {
  private static Drivetrain mInstance;

  // 3 meters per second.
  public static final double kMaxSpeed = 1.0;

  // 3 meters per second.
  public static final double kMaxAcceleration = 2.0;

  // 1/2 rotation per second.
  public static final double kMaxAngularSpeed = Math.PI * 0.7;

  private static final double kTrackWidth = 0.381 * 2;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = -4096;

  private static final double kSlowModeRotScale = 0.1;

  private final SimulatableCANSparkMax mLeftLeader = new SimulatableCANSparkMax(Constants.kDrivetrainFLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mLeftFollower = new SimulatableCANSparkMax(Constants.kDrivetrainBLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightLeader = new SimulatableCANSparkMax(Constants.kDrivetrainFRMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightFollower = new SimulatableCANSparkMax(Constants.kDrivetrainBRMotorId,
      MotorType.kBrushless);

  private final MotorControllerGroup mLeftGroup = new MotorControllerGroup(mLeftLeader, mLeftFollower);
  private final MotorControllerGroup mRightGroup = new MotorControllerGroup(mRightLeader, mRightFollower);

  private final Encoder mLeftEncoder = new Encoder(0, 1);
  private final Encoder mRightEncoder = new Encoder(2, 3);

  private final PIDController mLeftPIDController = new PIDController(0, 0, 0);
  private final PIDController mRightPIDController = new PIDController(0, 0, 0);

  private final AnalogGyro mGyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(),
      mLeftEncoder.getDistance(), mRightEncoder.getDistance());

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation classes help us simulate our robot
  private final AnalogGyroSim mGyroSim = new AnalogGyroSim(mGyro);
  private final EncoderSim mLeftEncoderSim = new EncoderSim(mLeftEncoder);
  private final EncoderSim mRightEncoderSim = new EncoderSim(mRightEncoder);
  private final Field2d mFieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> mDrivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim mDrivetrainSimulator = new DifferentialDrivetrainSim(
      mDrivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

  public static Drivetrain getInstance() {
    if (mInstance == null) {
      mInstance = new Drivetrain();
    }
    return mInstance;
  }

  private Drivetrain() {
    mLeftLeader.restoreFactoryDefaults();
    mLeftLeader.setIdleMode(IdleMode.kCoast);
    mLeftFollower.restoreFactoryDefaults();
    mLeftFollower.setIdleMode(IdleMode.kCoast);
    mRightLeader.restoreFactoryDefaults();
    mRightLeader.setIdleMode(IdleMode.kCoast);
    mRightFollower.restoreFactoryDefaults();
    mRightFollower.setIdleMode(IdleMode.kCoast);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    mRightGroup.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    mLeftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    mRightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    mLeftEncoder.reset();
    mRightEncoder.reset();

    mRightGroup.setInverted(true);
    SmartDashboard.putData("Field", mFieldSim);
  }

  private boolean mSlowMode = false;

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = mFeedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = mFeedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = mLeftPIDController.calculate(mLeftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = mRightPIDController.calculate(mRightEncoder.getRate(), speeds.rightMetersPerSecond);

    mLeftGroup.setVoltage(leftOutput + leftFeedforward);
    mRightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void slowMode(boolean slow) {
    mSlowMode = slow;
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot    the rotation
   */
  public void drive(double xSpeed, double rot) {
    if (mSlowMode) {
      setSpeeds(mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot * kSlowModeRotScale)));
    } else {
      setSpeeds(mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    mOdometry.update(
        mGyro.getRotation2d(), mLeftEncoder.getDistance(), mRightEncoder.getDistance());
  }

  public void zeroSensors() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    mLeftEncoder.reset();
    mRightEncoder.reset();
    mDrivetrainSimulator.setPose(pose);

    mOdometry.resetPosition(pose.getRotation(), mLeftEncoder.getDistance(),
        mRightEncoder.getDistance(), pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    mDrivetrainSimulator.setInputs(
        mLeftGroup.get() * RobotController.getInputVoltage(),
        mRightGroup.get() * RobotController.getInputVoltage());
    mDrivetrainSimulator.update(0.02);

    mLeftEncoderSim.setDistance(mDrivetrainSimulator.getLeftPositionMeters());
    mLeftEncoderSim.setRate(mDrivetrainSimulator.getLeftVelocityMetersPerSecond());
    mRightEncoderSim.setDistance(mDrivetrainSimulator.getRightPositionMeters());
    mRightEncoderSim.setRate(mDrivetrainSimulator.getRightVelocityMetersPerSecond());
    mGyroSim.setAngle(-mDrivetrainSimulator.getHeading().getDegrees());
  }

  @Override
  public void periodic() {
  }

  @Override
  public void stop() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
    updateOdometry();
    mFieldSim.setRobotPose(getPose());
  }
}
