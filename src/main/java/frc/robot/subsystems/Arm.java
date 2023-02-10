package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class Arm extends Subsystem {

  private static Arm arm = null;

  private static final double kShoulderMotorP = 1.0;
  private static final double kShoulderMotorI = 0.0;
  private static final double kShoulderMotorD = 0.0;

  private static final double kElbowMotorP = 1.0;
  private static final double kElbowMotorI = 0.0;
  private static final double kElbowMotorD = 0.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  private static final double kShoulderDegreesPerPulse = 2.0 * Math.PI / 4096;
  private static final double kElbowDegreesPerPulse = 2.0 * Math.PI / 4096;

  private static final double kShoulderGearRatio = 24.0;
  private static final double kElbowGearRatio = 24.0;

  private static final double kShoulderMass = Units.lbsToKilograms(15.0);
  private static final double kElbowMass = Units.lbsToKilograms(10.0);
  private static final double kShoulderLength = Units.inchesToMeters(30);
  private static final double kElbowLength = Units.inchesToMeters(20);

  private final DCMotor mShoulderGearbox = DCMotor.getNEO(1);
  private final DCMotor mElbowGearbox = DCMotor.getNEO(1);

  private final PIDController mShoulderPID = new PIDController(kShoulderMotorP, kShoulderMotorI, kShoulderMotorD);
  private final PIDController mElbowPID = new PIDController(kElbowMotorP, kElbowMotorI, kElbowMotorD);

  private final CANSparkMax mShoulderMotor = new CANSparkMax(Constants.Arm.Shoulder.kMotorId, MotorType.kBrushless);
  private final CANSparkMax mElbowMotor = new CANSparkMax(Constants.Arm.Elbow.kMotorId, MotorType.kBrushless);

  private final SingleJointedArmSim mShoulderSimulator = new SingleJointedArmSim(
      mShoulderGearbox,
      kShoulderGearRatio,
      SingleJointedArmSim.estimateMOI(kShoulderMass, kShoulderLength),
      kShoulderLength,
      Constants.Arm.Shoulder.kMinAngle,
      Constants.Arm.Shoulder.kMaxAngle,
      true);

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
  }

  @Override
  public void periodic() {
    DCMotor.getNEO(1);
    // TODO Auto-generated method stub

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
    // TODO Auto-generated method stub

  }
}
