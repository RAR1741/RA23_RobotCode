package frc.robot.autonomous;

import java.util.ArrayList;

import frc.robot.Robot;

public class AutoRunner {
  private static AutoRunner m_autoRunner = null;
  private ArrayList<AutoTask> m_tasks;

  private Robot m_robot;

  private AutoRunner(Robot robot) {
    m_robot = robot;
    m_tasks = new ArrayList<>();
  }

  public static AutoRunner getInstance(Robot robot) {
    if (m_autoRunner == null) {
      m_autoRunner = new AutoRunner(robot);
    }
    return m_autoRunner;
  }

  public AutoTask getNextTask() {
    // Pop the first task off the list
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueDoNothing() {
    m_tasks.add(new AutoTask(m_robot) {
      @Override
      public void start() {
        System.out.println("Starting do nothing auto...");
      }

      @Override
      public boolean run() {
        System.out.println("Do nothing auto complete");
        return true;
      }
    });
  }

  public void queueBlueDefaultTasks() {
    m_tasks.add(new AutoTask(m_robot) {
      @Override
      public void start() {
        System.out.println("1: Starting...");
      }

      @Override
      public boolean run() {
        System.out.println("1: Running...");
        return false;
      }
    });

    m_tasks.add(new AutoTask(m_robot) {
      @Override
      public void start() {
        System.out.println("2: Starting...");
      }

      @Override
      public boolean run() {
        System.out.println("2: Running...");
        return false;
      }
    });
  }
}
