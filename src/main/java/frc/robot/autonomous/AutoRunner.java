package frc.robot.autonomous;

import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.BlueDefaultMode;
import frc.robot.autonomous.modes.BlueRight_OneCubeHigh_Balance;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.RedCenter_OneCubeHigh_Balance;
import frc.robot.autonomous.modes.RedRight_OneCubeHigh_Balance;
import frc.robot.autonomous.modes.RedLeft_OneCubeHigh_Balance;
import frc.robot.autonomous.tasks.Task;

public class AutoRunner {
  private static AutoRunner m_autoRunner = null;
  private AutoModeBase m_autoMode;

  public static AutoRunner getInstance() {
    if (m_autoRunner == null) {
      m_autoRunner = new AutoRunner();
    }
    return m_autoRunner;
  }

  public enum AutoMode {
    DO_NOTHING,
    BLUE_DEFAULT,
    RED_RIGHT_CUBE_BALANCE,
    BLUE_RIGHT_CUBE_BALANCE,
    RED_CENTER_CUBE_BALANCE,
    RED_LEFT_CUBE_BALANCE
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void setAutoMode(AutoMode mode) {
    switch (mode) {
      case DO_NOTHING:
        m_autoMode = new DoNothingMode();
        break;
      case BLUE_DEFAULT:
        m_autoMode = new BlueDefaultMode();
        break;
      case RED_RIGHT_CUBE_BALANCE:
        m_autoMode = new RedRight_OneCubeHigh_Balance();
        break;
      case BLUE_RIGHT_CUBE_BALANCE:
        m_autoMode = new BlueRight_OneCubeHigh_Balance();
        break;
      case RED_CENTER_CUBE_BALANCE:
        m_autoMode = new RedCenter_OneCubeHigh_Balance();
        break;
      case RED_LEFT_CUBE_BALANCE:
        m_autoMode = new RedLeft_OneCubeHigh_Balance();
        break;
      default:
        System.out.println("Invalid auto mode selected. Defaulting to do nothing.");
        m_autoMode = new DoNothingMode();
        break;
    }

    m_autoMode.queueTasks();
  }
}
