package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {

  private boolean m_hatLock = false;

  private final DPadButton m_up = new DPadButton(this, DPadButton.Direction.UP);
  private final DPadButton m_down = new DPadButton(this, DPadButton.Direction.DOWN);
  private final DPadButton m_left = new DPadButton(this, DPadButton.Direction.LEFT);
  private final DPadButton m_right = new DPadButton(this, DPadButton.Direction.RIGHT);

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsDefaultState() {
    return this.getRawButton(2);
  }

  public boolean getHatUp() {
    if(this.getPOV() != -1) {
      if(!m_hatLock) {
        m_hatLock = m_up.get();
        return m_hatLock;
      }
    } else {
      m_hatLock = false;
    }
    return false;
  }

  public boolean getHatDown() {
    if(this.getPOV() != -1) {
      if(!m_hatLock) {
        m_hatLock = m_down.get();
        return m_hatLock;
      }
    } else {
      m_hatLock = false;
    }
    return false;
  }

  public boolean getHatLeft() {
    if(this.getPOV() != -1) {
      if(!m_hatLock) {
        m_hatLock = m_left.get();
        return m_hatLock;
      }
    } else {
      m_hatLock = false;
    }
    return false;
  }

  public boolean getHatRight() {
    if(this.getPOV() != -1) {
      if(!m_hatLock) {
        m_hatLock = m_right.get();
        return m_hatLock;
      }
    } else {
      m_hatLock = false;
    }
    return false;
  }

  public boolean getWantsMaxMovement() {
    return this.getRawButton(3);
  }

  public boolean getWantsGroundPosition() {
    return this.getRawButton(1);
  }

public boolean getWantsGripToggle() {
    return this.getRawButton(5);
}
}
