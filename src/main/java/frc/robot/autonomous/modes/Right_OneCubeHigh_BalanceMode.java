package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.AutoBalanceTask;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class Right_OneCubeHigh_BalanceMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.655021228445234, 4.458172598636864, Rotation2d.fromDegrees(180.0));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5)));

    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    queueTask(new GripperTask(false));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      queueTask(new ParallelTask(
          new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
          new DriveTrajectoryTask("RightFarBalance", 3.0, 1.5)));
    } else {
      queueTask(new ParallelTask(
          new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
          new DriveTrajectoryTask("RightFarBalance-B", 3.0, 1.5)));
    }

    // queueTask(new DriveForwardTask(2.0, -1.0));

    queueTask(new AutoBalanceTask());

    queueTask(new BrakeTask(true));
  }

}
