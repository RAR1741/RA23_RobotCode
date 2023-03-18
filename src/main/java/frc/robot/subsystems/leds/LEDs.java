package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = Constants.LEDs.k_totalLength;

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    m_led = new AddressableLED(Constants.LEDs.k_PWMId);
    m_led.setLength(m_ledTotalLength);
    m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    setArmRightColorMode(LEDModes.setColor(Color.kRed));
    setArmLeftColorMode(LEDModes.setColor(Color.kBlue));
    setDriveColorMode(LEDModes.rainbow);

    m_led.setData(m_buffer);
  }

  public void setArmRightColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> callback) {
    m_buffer = callback.apply(Constants.LEDs.ArmRight.k_start).apply(Constants.LEDs.ArmRight.k_length)
        .apply(m_buffer);
  }

  public void setArmLeftColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> callback) {
    m_buffer = callback.apply(Constants.LEDs.ArmLeft.k_start).apply(Constants.LEDs.ArmLeft.k_length)
        .apply(m_buffer);
  }

  public void setDriveColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> callback) {
    m_buffer = callback.apply(Constants.LEDs.Drive.k_start).apply(Constants.LEDs.Drive.k_length).apply(m_buffer);
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
