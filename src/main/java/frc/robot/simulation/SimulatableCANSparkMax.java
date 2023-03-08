package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableCANSparkMax extends CANSparkMax {
  SimDeviceSim m_CANSparkMaxSim;

  SimDouble m_CANSparkMaxSimAppliedOutput;
  SimDouble m_CANSparkMaxSimAnalogVoltage;

  public SimulatableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    m_CANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceId);
    m_CANSparkMaxSimAppliedOutput = m_CANSparkMaxSim.getDouble("Applied Output");
    m_CANSparkMaxSimAnalogVoltage = m_CANSparkMaxSim.getDouble("Analog Voltage");

    // TODO: Add other simulation fields
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // TODO: Figure out why this is mad when running on a real robot
    m_CANSparkMaxSimAppliedOutput.set(speed);
  }

  @Override
  public double get() {
    if (RobotBase.isReal()) {
      return super.get();
    } else {
      // TODO: figure out speed from voltage
      return m_CANSparkMaxSimAppliedOutput.get();
    }
  }

  @Override
  public void setVoltage(double voltage) {
    super.setVoltage(voltage);

    m_CANSparkMaxSimAnalogVoltage.set(voltage);
  }
}
