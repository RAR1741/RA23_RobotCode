package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class Limelight extends Subsystem {
  private static Limelight m_limelight;
  private NetworkTable m_limelightTable;
  private HashMap<String, Object> m_limelightInfo;
  private String m_name = "limelight";

  // Limelight Offsets:
  // X: 12 5426 in
  // Y: 9.19 in
  // Z: 10.1175 in

  /**
   * Constructor
   */
  private Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
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
    return toFieldPose(LimelightHelpers.getBotPose2d(m_name));
  }

  /**
   * Get whether there is a visible AprilTag
   *
   * @return If there is a visible AprilTag
   */
  public boolean seesAprilTag() {
    return m_limelightTable.getEntry("tv").getInteger(0) == 1;
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
  }

  /**
   * Converts the limelight coordinate system to the field coordinate system.
   *
   * @param pose Position of the robot
   * @return The position of the robot in terms of the field.
   */
  private Pose2d toFieldPose(Pose2d pose) {
    return pose.relativeTo(new Pose2d(-8.2296, -8.2296 / 2, Rotation2d.fromDegrees(0)));
  }
}
