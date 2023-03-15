package frc.robot.autonomous.modes;

import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;

public class BlueDefaultMode extends AutoModeBase {

  public void queueTasks() {
    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));
  }
}
