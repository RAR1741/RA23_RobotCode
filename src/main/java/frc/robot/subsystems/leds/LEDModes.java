package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDModes {
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> red = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        for (int i = start; i < (start + length); i++) {
          buffer.setLED(i, Color.kRed);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> blue = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        for (int i = start; i < (start + length); i++) {
          buffer.setLED(i, Color.kBlue);
        }
        return buffer;
      };
    };
  };

  private static double rainbowSpeed = 100;
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final var hue = (firstPixelHue + (i * 180 / length)) % 180;
          buffer.setHSV(i, hue, 255, 128);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> off = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        for (int i = start; i < (start + length); i++) {
          buffer.setLED(i, Color.kBlack);
        }
        return buffer;
      };
    };
  };

  // TODO: Add "pulse" mode

  // TODO: Add mode that takes in a color and sets all the leds to that color
}