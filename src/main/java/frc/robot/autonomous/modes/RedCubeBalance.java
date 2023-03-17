package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.AutoBalanceTask;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointModulesInwardTask;

public class RedCubeBalance extends AutoModeBase {
  @Override
  public Pose2d getStartingPosition() {
    return new Pose2d(14.7, 2.8, Rotation2d.fromDegrees(180.0));
  }

  public void queueTasks() {
    queueTask(new DriveForwardTask(0, 0));

    // queueTask(new
    // ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    // queueTask(new WaitTask(1.0));

    // queueTask(new GripperTask(false));

    // queueTask(new WaitTask(1.0));

    queueTask(
        new ParallelTask(
            new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
            new DriveForwardTask(2.3, 1))); // 6 meters is past

    // queueTask(new DriveForwardTask(0.5, 0.6));

    queueTask(new AutoBalanceTask());

    queueTask(new BrakeTask(true));

    queueTask(new PointModulesInwardTask());

    // queueTask(new WaitTask(0.5));

    // queueTask(new DriveForwardTask(-1.0, 0.05));
  }

}
