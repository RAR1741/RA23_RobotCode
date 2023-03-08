package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ArmSim {
  private static ArmSim m_armSim = null;

  private static final double k_shoulderSimOffset = 90;
  private static final double k_elbowSimOffset = 180;

  private static final double k_shoulderGearRatio = 54.0;
  private static final double k_elbowGearRatio = 36.0;
  private static final double k_wristGearRatio = 70.0;

  private final DCMotor m_shoulderGearbox = DCMotor.getNEO(1);
  private final DCMotor m_elbowGearbox = DCMotor.getNEO(1);
  private final DCMotor m_wristGearbox = DCMotor.getNeo550(1);

  // private final PWMSparkMax m_shoulderMotor = new PWMSparkMax(1); //ew pwm
  // private final PWMSparkMax m_elbowMotor = new PWMSparkMax(2);
  // private final PWMSparkMax m_wristMotor = new PWMSparkMax(3);
  private final SingleJointedArmSim m_shoulderSim = new SingleJointedArmSim(
      m_shoulderGearbox,
      k_shoulderGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Shoulder.k_length),
          Constants.Arm.Shoulder.k_mass), // TODO: include the mass of the 2nd arm
      Units.inchesToMeters(Constants.Arm.Shoulder.k_length),
      Constants.Arm.Shoulder.k_minAngle,
      Constants.Arm.Shoulder.k_maxAngle,
      true);

  private final SingleJointedArmSim m_elbowSim = new SingleJointedArmSim(
      m_elbowGearbox,
      k_elbowGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Elbow.k_length),
          Constants.Arm.Elbow.k_mass),
      Units.inchesToMeters(Constants.Arm.Elbow.k_length),
      Constants.Arm.Elbow.k_minAngle,
      Constants.Arm.Elbow.k_maxAngle,
      true);

  private final SingleJointedArmSim m_wristSim = new SingleJointedArmSim(
      m_wristGearbox,
      k_wristGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Arm.Wrist.k_length),
          Constants.Arm.Wrist.k_mass),
      Units.inchesToMeters(Constants.Arm.Wrist.k_length),
      Constants.Arm.Wrist.k_minAngle,
      Constants.Arm.Wrist.k_maxAngle,
      true);

  // This isn't a real encoder...
  // private final Encoder m_shoulderEncoder = new Encoder(20, 21);
  // private final EncoderSim m_shoulderEncoderSim = new
  // EncoderSim(m_shoulderEncoder);
  // private final EncoderSim m_elbowEncoderSim = new EncoderSim(m_elbowEncoder);
  // private final EncoderSim m_wristEncoderSim = new EncoderSim(m_wristEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmBase and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(Constants.Simulation.k_width, Constants.Simulation.k_height);

  // Center of arm coordinate system
  private final Translation2d m_origin = new Translation2d(Constants.Simulation.k_width / 2, 0);

  private final MechanismRoot2d m_shoulderPivot = m_mech2d.getRoot("ArmShoulderPivot", m_origin.getX(),
      Constants.Arm.k_shoulderPivotHeight);

  private final MechanismLigament2d m_armBase = m_shoulderPivot.append(
      new MechanismLigament2d(
          "ArmBase",
          Constants.Arm.k_shoulderPivotHeight,
          -90,
          4,
          new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d m_arm1 = m_shoulderPivot.append(
      new MechanismLigament2d(
          "Arm1",
          Constants.Arm.Shoulder.k_length,
          k_shoulderSimOffset,
          4,
          new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d m_arm2 = m_arm1.append(
      new MechanismLigament2d(
          "Arm2",
          Constants.Arm.Elbow.k_length,
          k_elbowSimOffset,
          4,
          new Color8Bit(Color.kGreen)));

  private final MechanismRoot2d m_crosshair = m_mech2d.getRoot("Crosshair", m_origin.getX(), m_origin.getY());

  public static ArmSim getInstance() {
    if (m_armSim == null) {
      m_armSim = new ArmSim();
    }
    return m_armSim;
  }

  private ArmSim() {
    addAdditionalDrawings();

    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  public void updateArmPosition(double shoulderAngle, double elbowAngle, double wristAngle, double x, double y) {
    m_arm1.setAngle(k_shoulderSimOffset - shoulderAngle);
    m_arm2.setAngle(k_elbowSimOffset + elbowAngle + shoulderAngle);
    Translation2d setpoint = m_origin.plus(new Translation2d(x, y));

    m_crosshair.setPosition(setpoint.getX(), setpoint.getY());

    SmartDashboard.putNumberArray("Arm Sim Angles", new double[] { shoulderAngle, elbowAngle });
    // TODO: add wrist display... somehow...
  }

  public double[] getArmAngles() {
    double simShoulderAngle = m_arm1.getAngle();
    double simElbowAngle = m_arm2.getAngle();
    double shoulderAngle = k_shoulderSimOffset - simShoulderAngle;
    double elbowAngle = simElbowAngle - k_elbowSimOffset - shoulderAngle;
    return new double[] { shoulderAngle, elbowAngle };
  }

  // TODO I don't really understand this and didn't want to break anything, so I
  // just disabled it
  /*
   * @Override
   * public void periodic() {
   * // In this method, we update our simulation of what our arm is doing
   * // First, we set our "inputs" (voltages)
   * m_shoulderSim.setInput(m_shoulderMotor.get() *
   * RobotController.getBatteryVoltage());
   * m_elbowSim.setInput(m_elbowMotor.get() *
   * RobotController.getBatteryVoltage());
   *
   * // Next, we update it. The standard loop time is 20ms.
   * m_shoulderSim.update(0.020);
   * m_elbowSim.update(0.020);
   *
   * // Finally, we set our simulated encoder's readings and simulated battery
   * // voltage
   * m_shoulderEncoderSim.setDistance(m_shoulderSim.getAngleRads());
   * m_elbowEncoderSim.setDistance(m_elbowSim.getAngleRads());
   *
   * // SimBattery estimates loaded battery voltages RoboRioSim.setVInVoltage(
   * BatterySim.calculateDefaultBatteryLoadedVoltage(
   * m_shoulderSim.getCurrentDrawAmps() + m_elbowSim.getCurrentDrawAmps());
   *
   * m_periodicIO.shoulderAngle = Preferences.getDouble("shoulderAngle",
   * m_periodicIO.shoulderAngle);
   * m_periodicIO.elbowAngle = Preferences.getDouble("elbowAngle",
   * m_periodicIO.elbowAngle);
   *
   * double shoulderPIDOutput =
   * m_shoulderPID.calculate(m_shoulderEncoder.getDistance(),
   * Units.degreesToRadians(m_periodicIO.shoulderAngle));
   * double elbowPIDOutput = m_elbowPID.calculate(m_elbowEncoder.getDistance(),
   * Units.degreesToRadians(m_periodicIO.elbowAngle));
   *
   * SmartDashboard.putNumber("Shoulder Diff",
   * Units.radiansToDegrees(m_shoulderEncoder.getDistance() -
   * Units.degreesToRadians(m_periodicIO.shoulderAngle)));
   * SmartDashboard.putNumber("Elbow Diff",
   * Units.radiansToDegrees(m_elbowEncoder.getDistance() -
   * Units.degreesToRadians(m_periodicIO.elbowAngle)));
   *
   * m_shoulderMotor.setVoltage(shoulderPIDOutput);
   * m_elbowMotor.setVoltage(elbowPIDOutput);
   *
   * // Update the Mechanism Arm angle based on the simulated arm angle
   * m_arm1.setAngle(Units.radiansToDegrees(m_shoulderSim.getAngleRads()));
   * m_arm2.setAngle(Units.radiansToDegrees(m_elbowSim.getAngleRads()));
   * }
   */

  private void addAdditionalDrawings() {
    // Draw the robot's bumpers
    double bumperPosition = Constants.Simulation.k_width / 2 - Constants.Robot.k_length / 2;
    m_mech2d.getRoot("Robot", bumperPosition, Constants.Robot.k_bumperStart).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Robot.k_length,
            0,
            Constants.Robot.k_bumperHeight,
            new Color8Bit(Color.kRed)));

    // Draw the scoring grid base
    double gridStartPosition = Constants.Simulation.k_width / 2 + Constants.Robot.k_length / 2;
    m_mech2d.getRoot("GridGround", gridStartPosition, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.k_highGoalX,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw the low goal
    m_mech2d.getRoot("GridLowGoal", gridStartPosition + Constants.Field.k_lowGoalX, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.k_lowGoalHeight,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw the high goal
    m_mech2d.getRoot("GridHighGoal", gridStartPosition + Constants.Field.k_highGoalX, 0).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Field.k_highGoalHeight,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    // Have to draw the crosshair in a weird way because each "ligament" has to be
    // connected
    // (i.e. 4 lines radially from the center rather than two crossed lines)
    // TODO: Factor this out

    final double k_crosshairLength = 2;
    final double k_crosshairThickness = 1;
    final Color8Bit crosshairColor = new Color8Bit(Color.kOrange);

    final String[] directions = { "Right", "Top", "Left", "Bottom" };
    double crosshairAngle = 0;
    for (String direction : directions) {
      m_crosshair.append(
          new MechanismLigament2d(
              "Crosshair" + direction,
              k_crosshairLength,
              crosshairAngle,
              k_crosshairThickness,
              crosshairColor));
      crosshairAngle += 90;
    }
  }
}
