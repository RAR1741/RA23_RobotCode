package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

// Custom yellow buffer.setRGB(i, 255, (int) (255 * 0.50), 0);

public final class LEDModes {
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> setColor(
      Color color) {
    return (
        start) -> {
      return (length) -> {
        return (buffer) -> {
          for (int i = start; i < (start + length); i++) {
            buffer.setLED(i, color);
          }
          return buffer;
        };
      };
    };
  }

  private static double rainbowSpeed = 100;
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = (firstPixelHue + (i * 180 / length)) % 180;
          buffer.setHSV(i, hue, 255, 128);
        }
        return buffer;
      };
    };
  };

  private static int clamp(int val, int min, int max) {
    return Math.max(min, Math.min(max, val));
  }

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redBreathe = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = clamp((firstPixelHue + (i * 180 / length)) % 180, 51, 255);
          buffer.setRGB(i, hue, 0, 0);
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
