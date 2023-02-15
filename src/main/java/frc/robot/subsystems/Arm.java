package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class Arm extends Subsystem {

  private static Arm arm = null;

  private static final double kShoulderMotorP = 50.0;
  private static final double kShoulderMotorI = 2.0;
  private static final double kShoulderMotorD = 5.0;

  private static final double kElbowMotorP = 50.0;
  private static final double kElbowMotorI = 2.0;
  private static final double kElbowMotorD = 5.0;

  private static final double kShoulderGearRatio = 54.0;
  private static final double kElbowGearRatio = 36.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  private static final double kShoulderDegreesPerPulse = 2.0 * Math.PI / 4096.0;
  private static final double kElbowDegreesPerPulse = 2.0 * Math.PI / 4096.0;

  private final DCMotor mShoulderGearbox = DCMotor.getNEO(1);
  private final DCMotor mElbowGearbox = DCMotor.getNEO(1);

  private final PIDController mShoulderPID = new PIDController(kShoulderMotorP, kShoulderMotorI, kShoulderMotorD);
  private final PIDController mElbowPID = new PIDController(kElbowMotorP, kElbowMotorI, kElbowMotorD);

  // TOOD: figure out how to use the SparkMax
  private final PWMSparkMax mShoulderMotor = new PWMSparkMax(1);
  // private final SimulatableCANSparkMax mShoulderMotor = new
  // SimulatableCANSparkMax(Constants.Arm.Shoulder.kMotorId,
  // MotorType.kBrushless);

  private final PWMSparkMax mElbowMotor = new PWMSparkMax(2);
  // private final SimulatableCANSparkMax mElbowMotor = new
  // SimulatableCANSparkMax(Constants.Arm.Elbow.kMotorId,
  // MotorType.kBrushless);

  private final SingleJointedArmSim mShoulderSim = new SingleJointedArmSim(
      mShoulderGearbox,
      kShoulderGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Shoulder.kLength),
          Constants.Arm.Shoulder.kMass), // TODO: include the mass of the 2nd arm
      Units.inchesToMeters(Constants.Arm.Shoulder.kLength),
      Constants.Arm.Shoulder.kMinAngle,
      Constants.Arm.Shoulder.kMaxAngle,
      true);

  private final SingleJointedArmSim mElbowSim = new SingleJointedArmSim(
      mElbowGearbox,
      kElbowGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Elbow.kLength),
          Constants.Arm.Elbow.kMass),
      Units.inchesToMeters(Constants.Arm.Elbow.kLength),
      Constants.Arm.Elbow.kMinAngle,
      Constants.Arm.Elbow.kMaxAngle,
      true);

  // This isn't a real encoder...
  private final Encoder mShoulderEncoder = new Encoder(20, 21);
  private final EncoderSim mShoulderEncoderSim = new EncoderSim(mShoulderEncoder);

  private final Encoder mElbowEncoder = new Encoder(22, 23);
  private final EncoderSim mElbowEncoderSim = new EncoderSim(mElbowEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmBase and moving Arm.
  private final Mechanism2d mMech2d = new Mechanism2d(Constants.Simulation.kWidth, Constants.Simulation.kHeight);

  private final MechanismRoot2d mShoulderPivot = mMech2d.getRoot("ArmShoulderPivot", Constants.Simulation.kWidth / 2,
      Constants.Arm.kShoulderPivotHeight);

  private final MechanismLigament2d mArmBase = mShoulderPivot.append(
      new MechanismLigament2d(
          "ArmBase",
          Constants.Arm.kShoulderPivotHeight,
          -90,
          4,
          new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d mArm1 = mShoulderPivot.append(
      new MechanismLigament2d(
          "Arm1",
          Constants.Arm.Shoulder.kLength,
          Units.radiansToDegrees(mShoulderSim.getAngleRads()),
          4,
          new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d mArm2 = mArm1.append(
      new MechanismLigament2d(
          "Arm2",
          Constants.Arm.Elbow.kLength,
          Units.radiansToDegrees(mElbowSim.getAngleRads()),
          4,
          new Color8Bit(Color.kGreen)));

  private static class PeriodicIO {
    // Automated control
    public double shoulderAngle;
    public double elbowAngle;

    // Manual control
    public double shoulderMotorPower;
    public double elbowMotorPower;
  }

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  public static Arm getInstance() {
    if (arm == null) {
      arm = new Arm();
    }
    return arm;
  }

  private Arm() {
    mShoulderEncoder.setDistancePerPulse(kShoulderDegreesPerPulse);
    mElbowEncoder.setDistancePerPulse(kElbowDegreesPerPulse);

    addAdditionalDrawings();

    SmartDashboard.putData("Arm Sim", mMech2d);

    if (!Preferences.containsKey("shoulderAngle")) {
      Preferences.setDouble("shoulderAngle", mPeriodicIO.shoulderAngle);
    }
    if (!Preferences.containsKey("elbowAngle")) {
      Preferences.setDouble("elbowAngle", mPeriodicIO.elbowAngle);
    }
  }

  private void addAdditionalDrawings() {
    // Draw the robot's bumpers
    double bumperPosition = Constants.Simulation.kWidth / 2 - Constants.Robot.kLength / 2;
    mMech2d.getRoot("Robot", bumperPosition, Constants.Robot.kBumperStart).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Robot.kLength,
            0,
            Constants.Robot.kBumperHeight,
            new Color8Bit(Color.kRed)));

    // Draw the scoring grid base
    double gridStartPosition = Constants.Simulation.kWidth / 2 + Constants.Robot.kLength / 2;
    mMech2d.getRoot("GridGround", gridStartPosition, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.kHighGoalX,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw the low goal
    mMech2d.getRoot("GridLowGoal", gridStartPosition + Constants.Field.kLowGoalX, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.kLowGoalHeight,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw the high goal
    mMech2d.getRoot("GridHighGoal", gridStartPosition + Constants.Field.kHighGoalX, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.kHighGoalHeight,
            90,
            5,
            new Color8Bit(Color.kWhite)));
  }

  @Override
  public void periodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    mShoulderSim.setInput(mShoulderMotor.get() * RobotController.getBatteryVoltage());
    mElbowSim.setInput(mElbowMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    mShoulderSim.update(0.020);
    mElbowSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    mShoulderEncoderSim.setDistance(mShoulderSim.getAngleRads());
    mElbowEncoderSim.setDistance(mElbowSim.getAngleRads());

    // SimBattery estimates loaded battery voltages RoboRioSim.setVInVoltage(
    BatterySim.calculateDefaultBatteryLoadedVoltage(
        mShoulderSim.getCurrentDrawAmps() + mElbowSim.getCurrentDrawAmps());

    mPeriodicIO.shoulderAngle = Preferences.getDouble("shoulderAngle", mPeriodicIO.shoulderAngle);
    mPeriodicIO.elbowAngle = Preferences.getDouble("elbowAngle", mPeriodicIO.elbowAngle);

    double shoulderPIDOutput = mShoulderPID.calculate(mShoulderEncoder.getDistance(),
        Units.degreesToRadians(mPeriodicIO.shoulderAngle));
    double elbowPIDOutput = mElbowPID.calculate(mElbowEncoder.getDistance(),
        Units.degreesToRadians(mPeriodicIO.elbowAngle));

    SmartDashboard.putNumber("Shoulder Diff",
        Units.radiansToDegrees(mShoulderEncoder.getDistance() - Units.degreesToRadians(mPeriodicIO.shoulderAngle)));
    SmartDashboard.putNumber("Elbow Diff",
        Units.radiansToDegrees(mElbowEncoder.getDistance() - Units.degreesToRadians(mPeriodicIO.elbowAngle)));

    mShoulderMotor.setVoltage(shoulderPIDOutput);
    mElbowMotor.setVoltage(elbowPIDOutput);

    // Update the Mechanism Arm angle based on the simulated arm angle
    mArm1.setAngle(Units.radiansToDegrees(mShoulderSim.getAngleRads()));
    mArm2.setAngle(Units.radiansToDegrees(mElbowSim.getAngleRads()));
  }

  @Override
  public void stop() {
    mShoulderMotor.set(0);
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub

  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
