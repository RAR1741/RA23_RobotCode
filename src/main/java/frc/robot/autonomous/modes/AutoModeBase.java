package frc.robot.autonomous.modes;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
    SwerveDrive.getInstance().setPose(getStartingPosition());
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public abstract Pose2d getStartingPosition();

  public abstract void queueTasks();
}
