package frc.robot.simulation;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableCANSparkMax extends CANSparkMax {
  SimDeviceSim mCANSparkMaxSim;

  SimDouble mCANSparkMaxSimAppliedOutput;
  SimDouble mCANSparkMaxSimAnalogVoltage;

  public SimulatableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    mCANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceId);
    mCANSparkMaxSimAppliedOutput = mCANSparkMaxSim.getDouble("Applied Output");
    mCANSparkMaxSimAnalogVoltage = mCANSparkMaxSim.getDouble("Analog Voltage");

    // TODO: Add other simulation fields
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // TODO: Figure out why this is mad when running on a real robot
    mCANSparkMaxSimAppliedOutput.set(speed);
  }

  @Override
  public void setVoltage(double voltage) {
    super.setVoltage(voltage);

    mCANSparkMaxSimAnalogVoltage.set(voltage);
  }
}
