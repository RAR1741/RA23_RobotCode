package frc.robot.autonomous;

import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.DefaultMode;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.Center_OneCubeHigh_BalanceMode;
import frc.robot.autonomous.modes.Center_OneCubeHigh_Balance_MobilityMode;
import frc.robot.autonomous.modes.Left_OneCubeHigh_BalanceMode;
import frc.robot.autonomous.modes.Right_OneCubeHigh_BalanceMode;
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
    DEFAULT,
    RIGHT_CUBE_BALANCE,
    CENTER_CUBE_BALANCE,
    LEFT_CUBE_BALANCE
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void setAutoMode(AutoMode mode) {
    switch (mode) {
      case DO_NOTHING:
        m_autoMode = new DoNothingMode();
        break;
      case DEFAULT:
        m_autoMode = new DefaultMode();
        break;
      case RIGHT_CUBE_BALANCE:
        m_autoMode = new Right_OneCubeHigh_BalanceMode();
        break;
      case CENTER_CUBE_BALANCE:
        m_autoMode = new Center_OneCubeHigh_Balance_MobilityMode();
        break;
      case LEFT_CUBE_BALANCE:
        m_autoMode = new Left_OneCubeHigh_BalanceMode();
        break;
      default:
        System.out.println("Invalid auto mode selected. Defaulting to do nothing.");
        m_autoMode = new DoNothingMode();
        break;
    }

    m_autoMode.queueTasks();
  }
}
