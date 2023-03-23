package frc.robot;

import java.io.File;
import java.util.Calendar;
import java.util.HashMap;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {

  private static Logger m_logger;
  private static DataLog m_log;

  private static HashMap<String, Integer> m_entries = new HashMap<>();
  // private static ArrayList<String> m_doubleEntries = new ArrayList<>();
  // private static ArrayList<String> m_booleanEntries = new ArrayList<>();

  public static Logger getInstance() {
    if (m_logger == null) {
      m_logger = new Logger();
    }
    return m_logger;
  }

  private Logger() {
    // Initialize on-board logging

    String dir;
    if (RobotBase.isReal()) {
      dir = "/home/lvuser/logs/";
    } else {
      // this.getClass()
      dir = Filesystem.getLaunchDirectory() + "\\logs";
    }

    new File(dir).mkdirs();

    String date = Calendar.YEAR + "_" + Calendar.MONTH + "_" + Calendar.DATE;
    int match = 1;
    File[] files = new File(dir).listFiles();
    for (File i : files) {
      if (i.getName().contains(date)) {
        match++;
      }
    }

    String fileName = "RAR_LOG-" + date + "-" + match + ".wpilog";
    DataLogManager.start(dir, fileName);
    
    m_log = DataLogManager.getLog();
    DriverStation.startDataLog(m_log); // Driver Station/Joystick Logs

    DataLogManager.log("Logging initialized. Fard."); // :)
  }

  public static void addEntry(String key, String data) {
    if (!m_entries.containsKey(key)) {
      m_log.start(key, "String");
      m_entries.put(key, m_log.start(key, "String"));
    } else {
      m_log.appendString(m_entries.get(key).intValue(), data, 0);
    }
  }

  public static void addEntry(String key, double data) {
    if (!m_entries.containsKey(key)) {
      m_log.start(key, "double");
      m_entries.put(key, m_log.start(key, "double"));
    } else {
      m_log.appendDouble(m_entries.get(key).intValue(), data, 0);
    }
  }

  public static void addEntry(String key, boolean data) {
    if (!m_entries.containsKey(key)) {
      m_log.start(key, "boolean");
      m_entries.put(key, m_log.start(key, "boolean"));
    } else {
      m_log.appendBoolean(m_entries.get(key).intValue(), data, 0);
    }
  }

  public static void addEntry(String key, double[] data) {
    if (!m_entries.containsKey(key)) {
      m_log.start(key, "double[]");
      m_entries.put(key, m_log.start(key, "double[]"));
    } else {
      m_log.appendDoubleArray(m_entries.get(key).intValue(), data, 0);
    }
  }
}
