package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class DefaultMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.655021228445234, 0, Rotation2d.fromDegrees(180));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5)));

    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    queueTask(new GripperTask(false));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    queueTask(
        new ParallelTask(
            new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
            new DriveForwardTask(5.0, 1.0)));

    queueTask(new BrakeTask(true));
  }
}
