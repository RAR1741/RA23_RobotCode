package frc.robot.autonomous;

import java.io.File;
import java.io.FilenameFilter;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AutoChooser {
  private static final String k_defaultAuto = "Default";

  PathPlannerTrajectory m_selectedAuto;

  private String m_selectedAutoName;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public AutoChooser() {
    // TODO: Make this an actual default auto
    // m_chooser.setDefaultOption("Default", k_defaultAuto);

    // Populate the chooser with all the available autos from the file system
    String[] pathnames = new File(Filesystem.getDeployDirectory(), "pathplanner/").list(new FilenameFilter() {
      @Override
      public boolean accept(File f, String name) {
        return name.endsWith(".path");
      }
    });

    for (String pathname : pathnames) {
      String optionName = pathname.replace(".path", "");
      m_chooser.addOption(optionName, optionName);
    }

    SmartDashboard.putData("Auto choices", m_chooser);
  }

  private void loadSelectedAuto() {
    try {
      m_selectedAuto = PathPlanner.loadPath(m_selectedAutoName,
          new PathConstraints(Constants.Auto.k_maxSpeed, Constants.Auto.k_maxAcceleration));
    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + m_selectedAutoName, ex.getStackTrace());
    }
  }

  private void updateSelectedAutoName() {
    m_selectedAutoName = m_chooser.getSelected();
  }

  public PathPlannerTrajectory getSelectedAuto() {
    updateSelectedAutoName();
    loadSelectedAuto();

    return m_selectedAuto;
  }

  public String selectedAutoName() {
    return m_selectedAutoName;
  }
}
