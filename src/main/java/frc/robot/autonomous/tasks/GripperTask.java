package frc.robot.autonomous.tasks;

import frc.robot.subsystems.arm.Arm;

public class GripperTask extends Task {
  private Arm m_arm;
  private boolean m_engaged;

  public GripperTask(boolean engaged) {
    m_arm = Arm.getInstance();
    m_engaged = engaged;
  }

  @Override
  public void start() {
    m_arm.setGripper(m_engaged);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
