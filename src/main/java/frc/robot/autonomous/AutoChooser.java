package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRunner.AutoMode;

public class AutoChooser {
  AutoMode m_selectedAuto;

  private String m_selectedAutoName;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public AutoChooser() {
    m_chooser.setDefaultOption("DEFAULT", "DEFAULT");

    // Populate the chooser with all the available autos
    for (AutoMode mode : AutoRunner.AutoMode.values()) {
      m_chooser.addOption(mode.name(), mode.name());
    }

    SmartDashboard.putData("Auto picker", m_chooser);
  }

  private void updateSelectedAutoName() {
    m_selectedAutoName = m_chooser.getSelected();
  }

  public AutoMode getSelectedAuto() {
    updateSelectedAutoName();
    m_selectedAuto = AutoRunner.AutoMode.valueOf(m_selectedAutoName);

    return m_selectedAuto;
  }

  public String selectedAutoName() {
    return m_selectedAutoName;
  }
}
