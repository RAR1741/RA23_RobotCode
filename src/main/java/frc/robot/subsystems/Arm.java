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
  private static final double kShoulderMotorI = 0.01;
  private static final double kShoulderMotorD = 0.0;

  // private static final double kElbowMotorP = 1.0;
  // private static final double kElbowMotorI = 0.0;
  // private static final double kElbowMotorD = 0.0;

  private static final double kShoulderGearRatio = 100.0;
  // private static final double kElbowGearRatio = 24.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  private static final double kShoulderDegreesPerPulse = 2.0 * Math.PI / 4096.0;
  // private static final double kShoulderDegreesPerPulse = 360.0 / 4096.0;
  // private static final double kElbowDegreesPerPulse = 2.0 * Math.PI / 4096;

  // private static final double kElbowMass = Units.lbsToKilograms(10.0);
  // private static final double kElbowLength = Units.inchesToMeters(20);

  private final DCMotor mShoulderGearbox = DCMotor.getNEO(1);
  // private final DCMotor mElbowGearbox = DCMotor.getNEO(1);

  private final PIDController mShoulderPID = new PIDController(kShoulderMotorP, kShoulderMotorI, kShoulderMotorD);
  // private final PIDController mElbowPID = new PIDController(kElbowMotorP,
  // kElbowMotorI, kElbowMotorD);

  // TOOD: figure out how to use the SparkMax
  private final PWMSparkMax mShoulderMotor = new PWMSparkMax(1);
  // private final SimulatableCANSparkMax mShoulderMotor = new
  // SimulatableCANSparkMax(Constants.Arm.Shoulder.kMotorId,
  // MotorType.kBrushless);

  // private final SimulatableCANSparkMax mElbowMotor = new
  // SimulatableCANSparkMax(Constants.Arm.Elbow.kMotorId,
  // MotorType.kBrushless);

  private final SingleJointedArmSim mShoulderSim = new SingleJointedArmSim(
      mShoulderGearbox,
      kShoulderGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Shoulder.kLength),
          Constants.Arm.Shoulder.kMass),
      Units.inchesToMeters(Constants.Arm.Shoulder.kLength),
      Constants.Arm.Shoulder.kMinAngle,
      Constants.Arm.Shoulder.kMaxAngle,
      true);

  // This isn't a real encoder...
  private final Encoder mShoulderEncoder = new Encoder(20, 21);
  private final EncoderSim mShoulderEncoderSim = new EncoderSim(mShoulderEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmBase and moving Arm.
  private final Mechanism2d mMech2d = new Mechanism2d(Constants.Arm.kSimulationWidth, Constants.Arm.kSimulationHeight);

  private final MechanismRoot2d mShoulderPivot = mMech2d.getRoot("ArmShoulderPivot", Constants.Arm.kSimulationWidth / 2,
      Constants.Arm.kShoulderPivotHeight);

  private final MechanismLigament2d mArmBase = mShoulderPivot.append(
      new MechanismLigament2d("ArmBase", Constants.Arm.kShoulderPivotHeight, -90));

  private final MechanismLigament2d mArm = mShoulderPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(mShoulderSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow)));

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

    SmartDashboard.putData("Arm Sim", mMech2d);

    mArmBase.setColor(new Color8Bit(Color.kBlue));

    if (!Preferences.containsKey("shoulderAngle")) {
      Preferences.setDouble("shoulderAngle", mPeriodicIO.shoulderAngle);
    }
  }

  @Override
  public void periodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    mShoulderSim.setInput(mShoulderMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    mShoulderSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    mShoulderEncoderSim.setDistance(mShoulderSim.getAngleRads());

    // SimBattery estimates loaded battery voltages RoboRioSim.setVInVoltage(
    BatterySim.calculateDefaultBatteryLoadedVoltage(mShoulderSim.getCurrentDrawAmps());

    mPeriodicIO.shoulderAngle = Preferences.getDouble("shoulderAngle", mPeriodicIO.shoulderAngle);

    // System.out.println("Shoulder Angle: " + mPeriodicIO.shoulderAngle);

    var pidOutput = mShoulderPID.calculate(mShoulderEncoder.getDistance(),
        Units.degreesToRadians(mPeriodicIO.shoulderAngle));

    SmartDashboard.putNumber("Distance Diff", mShoulderSim.getAngleRads() - mShoulderEncoder.getDistance());
    SmartDashboard.putNumber("Arm Diff",
        mShoulderEncoder.getDistance() - Units.degreesToRadians(mPeriodicIO.shoulderAngle));

    mShoulderMotor.setVoltage(pidOutput);

    // Update the Mechanism Arm angle based on the simulated arm angle
    mArm.setAngle(Units.radiansToDegrees(mShoulderSim.getAngleRads()));
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
