package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.AutoBalanceTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class Left_OneCubeHigh_BalanceMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.7, 1.05, Rotation2d.fromDegrees(180.0));
  }

  public void queueTasks() {

    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5)));

    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    queueTask(new WaitTask(1.0));

    queueTask(new GripperTask(false));

    queueTask(new WaitTask(1.0));

    // queueTask(new ParallelTask( // TODO: DriveForward not working in ParallelTask
    // new DriveTrajectoryTask("RedLeftFarBalance", 1.0, 0.5),
    // new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose())));

    queueTask(new ParallelTask(
        new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
        new DriveTrajectoryTask("LeftFarBalance", 2.0, 0.5)));

    queueTask(new DriveForwardTask(1.5, -1.0)); // TODO: Always goes forward in sim

    queueTask(new AutoBalanceTask());
  }

}
