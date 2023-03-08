package frc.robot;

import java.util.HashMap;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Logger {

  private static Logger m_logger;
  private static DataLog m_log;

  private static HashMap<String, StringLogEntry> m_stringEntries;
  private static HashMap<String, DoubleLogEntry> m_doubleEntries;
  private static HashMap<String, BooleanLogEntry> m_booleanEntries;

  public static Logger getInstance() {
    if (m_logger == null) {
      m_logger = new Logger();
    }
    return m_logger;
  }

  private Logger() {
    // Initialize on-board logging
    DataLogManager.start();
    m_log = DataLogManager.getLog();
    DataLogManager.log("Logging initialized. Fard."); // :)

    DriverStation.startDataLog(m_log); // Driver Station/Joystick Logs

    m_stringEntries = new HashMap<>();
    m_doubleEntries = new HashMap<>();
    m_booleanEntries = new HashMap<>();
  }

  public static void addEntry(String key, String data) {
    if (!m_stringEntries.containsKey(key)) {
      m_stringEntries.put(key, new StringLogEntry(m_log, key));
    }
    m_stringEntries.get(key).append(data);
  }

  public static void addEntry(String key, double data) {
    if (!m_doubleEntries.containsKey(key)) {
      m_doubleEntries.put(key, new DoubleLogEntry(m_log, key));
    }
    m_doubleEntries.get(key).append(data);
  }

  public static void addEntry(String key, boolean data) {
    if (!m_booleanEntries.containsKey(key)) {
      m_booleanEntries.put(key, new BooleanLogEntry(m_log, key));
    }
    m_booleanEntries.get(key).append(data);
  }
}
