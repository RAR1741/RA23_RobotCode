package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

public class Limelight extends Subsystem {
  private static Limelight m_limelight;
  private NetworkTable m_limelightTable;
  private HashMap<String, Object> m_limelightInfo;

  /**
   * Constructor
   */
  private Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_limelightInfo = new HashMap<String, Object>();
  }

  /**
   * Get a new instance of the limelight class
   *
   * @return New instance of the limelight class
   */
  public static Limelight getInstance() {
    if (m_limelight == null) {
      m_limelight = new Limelight();
    }
    return m_limelight;
  }

  /**
   * Enable the LEDs
   */
  public void setLightEnabled(boolean enabled) {
    m_limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1);
  }

  /**
   * Get the current bot position
   *
   * @return Current bot pose
   */
  public Pose2d getBotpose2D() {
    return LimelightHelpers.getBotPose2d("Limelight");
  }

  /**
   * Get whether there is a visible AprilTag
   *
   * @return If there is a visible AprilTag
   */
  public boolean seesAprilTag() {
    return (int) m_limelightTable.getEntry("tv").getInteger(0) == 1;
  }

  @Override
  public void periodic() {
    for (String key : this.m_limelightTable.getKeys()) {
      String type = this.m_limelightTable.getEntry(key).getType().name().substring(1);

      if (type.equals("String")) {
        this.m_limelightInfo.put(key, this.m_limelightTable.getEntry(key).getString("No value"));
      } else if (type.equals("Double")) {
        this.m_limelightInfo.put(key, this.m_limelightTable.getEntry(key).getDouble(Double.NaN));
      } else if (type.equals("DoubleArray")) {
        this.m_limelightInfo.put(key, this.m_limelightTable.getEntry(key).getDoubleArray(new double[6]));
      }

      if (type.equals("String") || type.equals("Double")) {
        SmartDashboard.putString(key, this.m_limelightInfo.get(key).toString());
      } else {
        SmartDashboard.putString(key, Arrays.toString((double[]) m_limelightInfo.get(key)));
      }
    }
  }

  @Override
  public void stop() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
  }
}
