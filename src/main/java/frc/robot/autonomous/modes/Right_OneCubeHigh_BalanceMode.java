package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.autonomous.tasks.WristTask;
import frc.robot.subsystems.arm.ArmPose;

public class Right_OneCubeHigh_BalanceMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.655021228445234, 4.458172598636864, Rotation2d.fromDegrees(180.0));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5),
        new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose())));

    // queueTask(new ParallelTask(
    // new ArmThrowTask(),
    // new WaitTask(0.75)));

    // queueTask(new
    // ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait + 0.2));

    queueTask(new GripperTask(false));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      // Start: 14.66, 4.46
      // End: 9.90, 4.41
      // Diff: -4.76, -0.05
      queueTask(new ParallelTask(
          new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
          new DriveTrajectoryTask("RightFar2Piece1", 2.0, 1.0, true)));
    } else {
      // Start: 1.88, 4.46
      // End: 6.64, 4.41
      // Diff: 4.76, -0.05
      queueTask(new ParallelTask(
          new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
          new DriveTrajectoryTask("RightFar2Piece1-B", 2.0, 1.0, true)));
    }

    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.FLOOR_PICKUP.getPose()));

    queueTask(new WaitTask(1.5));
    queueTask(new GripperTask(true));
    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    Pose2d tempPose = Constants.Arm.Preset.FLOOR_PICKUP.getPose();
    queueTask(new ArmTrajectoryTask(new ArmPose(tempPose.getX() + 12, tempPose.getY() + 10, tempPose.getRotation())));

    // queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()));

    queueTask(new ParallelTask(
        new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
        new WaitTask(Constants.Auto.k_defaultGripperWait)));

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      // Start: 9.90, 4.51
      // End: 14.83, 5.05
      // Diff: 4.93, 0.54
      queueTask(new ParallelTask(
          new WristTask(180.0),
          new DriveTrajectoryTask("RightFar2Piece2", 2.0, 1.0, false)));
    } else {
      // Start: 6.64, 4.51
      // End: 1.71, 5.05
      // Diff: -4.93, 0.54
      queueTask(new ParallelTask(
          new WristTask(180.0),
          new DriveTrajectoryTask("RightFar2Piece2-B", 2.0, 1.0, false)));
    }

    // queueTask(new ParallelTask(
    // new DriveForwardTask(-1, -0.4),
    // new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CONE.getPose())));

    // queueTask(new ParallelTask(
    //     new GripperTask(false),
    //     new WaitTask(Constants.Auto.k_defaultGripperWait)));

    // queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()));

    // queueTask(new DriveForwardTask(2.0, -1.0));

    // queueTask(new AutoBalanceTask());

    // queueTask(new BrakeTask(true));
  }
}
