package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.WaitTask;

public class DefaultMode extends AutoModeBase {
  @Override
  public Pose2d getStartingPosition() {
    return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  public void queueTasks() {
    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    queueTask(new GripperTask(false));

    queueTask(new WaitTask(1.0));

    queueTask(
        new ParallelTask(
            new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
            new DriveForwardTask(5.0, 1.0)));
  }
}
