package frc.robot.autonomous.tasks;

import frc.robot.subsystems.arm.Arm;

public class WristTask extends Task {
  private Arm m_arm;
  private double m_angleChange;

  public WristTask(double angleChange) {
    m_arm = Arm.getInstance();
    m_angleChange = angleChange;
  }

  @Override
  public void start() {
    m_arm.rotateWrist(m_angleChange);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
