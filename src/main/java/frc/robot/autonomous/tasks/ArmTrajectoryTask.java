package frc.robot.autonomous.tasks;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;

public class ArmTrajectoryTask extends Task {
  private Arm m_arm;
  private ArmPose m_pose;

  public ArmTrajectoryTask(ArmPose pose) {
    m_arm = Arm.getInstance();
    m_pose = pose;
  }

  @Override
  public void start() {
    m_arm.generateTrajectoryToPose(m_pose);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    boolean isRunning = m_arm.runTrajectory();
    return !isRunning;
  }
}
