package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.AutoBalanceTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class BlueRight_OneCubeHigh_Balance extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(1.9, 4.4, Rotation2d.fromDegrees(0.0)); // 1.9 4.4 0.0
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(1.0)));

    queueTask(new DriveTrajectoryTask("RedRightFarBalance", 1.0, 0.5));

    queueTask(new DriveForwardTask(2.0, 1.0));

    queueTask(new AutoBalanceTask());

    // queueTask(new DriveForwardTask(0, 0));

    // queueTask(new
    // ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    // queueTask(new WaitTask(1.0));

    // queueTask(new GripperTask(false));

    // queueTask(new WaitTask(1.0));

    // queueTask(
    // new ParallelTask(
    // new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
    // new DriveForwardTask(2.3, 1))); // 6 meters is past

    // queueTask(new DriveForwardTask(0.5, 0.6));

    // queueTask(new AutoBalanceTask());

    // queueTask(new BrakeTask(true));

    // queueTask(new WaitTask(0.5));

    // queueTask(new DriveForwardTask(-1.0, 0.05));
  }

}
