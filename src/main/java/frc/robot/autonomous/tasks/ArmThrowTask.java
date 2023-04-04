package frc.robot.autonomous.tasks;

import frc.robot.subsystems.arm.Arm;

public class ArmThrowTask extends Task {
  private Arm m_arm;

  public ArmThrowTask() {
    m_arm = Arm.getInstance();
  }

  @Override
  public void start() {
    m_arm.throwCube();
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    // boolean isRunning = m_arm.runTrajectory();
    // return !isRunning;
    return true;
  }
}
