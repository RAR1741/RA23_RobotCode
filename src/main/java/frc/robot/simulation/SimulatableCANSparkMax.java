package frc.robot.simulation;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableCANSparkMax extends CANSparkMax {
  SimDeviceSim m_CANSparkMaxSim;

  SimDouble m_CANSparkMaxSimAppliedOutput;

  public SimulatableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    m_CANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceId);
    m_CANSparkMaxSimAppliedOutput = m_CANSparkMaxSim.getDouble("Applied Output");

    // TODO: Add other simulation fields
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // TODO: Figure out why this is mad when running on a real robot
    // mCANSparkMaxSimAppliedOutput.set(speed);
  }
}
