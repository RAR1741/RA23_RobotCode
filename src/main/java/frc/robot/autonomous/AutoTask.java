package frc.robot.autonomous;

import frc.robot.Robot;

public class AutoTask {
  private final Robot m_robot;
  private boolean m_hasStarted = false;

  public AutoTask(Robot robot) {
    m_robot = robot;
  }

  public void start() {
  }

  public boolean run() {
    return true;
  }

  public AutoTask then(AutoTask nextTask) {
    return new AutoTask(m_robot) {
      @Override
      public void start() {
        AutoTask.this.start();
      }

      @Override
      public boolean run() {
        if (AutoTask.this.run()) {
          nextTask.start();
          return nextTask.run();
        }
        return false;
      }
    };
  }
}
