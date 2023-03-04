package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPose extends Pose2d {

  private final double k_x;
  private final double k_y;

  public ArmPose(double x, double y, Rotation2d rotation) {
    super(x, y, rotation);

    k_x = x;
    k_y = y;
  }

  public double[] getPosition() {
    return new double[] {k_x, k_y};
  }
}
