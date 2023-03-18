package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = Constants.LEDs.k_ledTotalLength;

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    m_led = new AddressableLED(Constants.LEDs.k_ledPWMId);
    m_led.setLength(m_ledTotalLength);
    m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    setDriveColorMode(LEDModes.red);
    setArmColorMode(LEDModes.rainbow);

    m_led.setData(m_buffer);
  }

  public void setDriveColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> callback) {
    callback.apply(Constants.LEDs.k_driveLEDStart).apply(Constants.LEDs.k_driveLEDLength).apply(m_buffer);
    m_led.setData(m_buffer);
  }

  public void setArmColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> callback) {
    callback.apply(Constants.LEDs.k_armLEDStart).apply(Constants.LEDs.k_armLEDLength).apply(m_buffer);
    m_led.setData(m_buffer);
  }

  @Override
  public void stop() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
  }
}
