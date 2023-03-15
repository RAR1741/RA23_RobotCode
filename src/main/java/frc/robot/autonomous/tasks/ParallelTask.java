package frc.robot.autonomous.tasks;

public class ParallelTask extends Task {
  private Task[] tasks;
  private boolean[] finished;
  private boolean allFinished = false;

  public ParallelTask(Task... tasks) {
    this.tasks = tasks;
    finished = new boolean[tasks.length];
  }

  @Override
  public void start() {
    for (Task task : tasks) {
      task.start();
    }
  }

  @Override
  public void update() {
    for (int i = 0; i < tasks.length; i++) {
      if (!finished[i]) {
        tasks[i].update();
        if (tasks[i].isFinished()) {
          finished[i] = true;
        }
      }
    }
    allFinished = true;
    for (boolean b : finished) {
      if (!b) {
        allFinished = false;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return allFinished;
  }

  @Override
  public void done() {
    for (Task task : tasks) {
      task.done();
    }
  }
}
