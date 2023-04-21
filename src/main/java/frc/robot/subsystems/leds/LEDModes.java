package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Helpers;

public final class LEDModes {
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> setColor(
      Color color) {
    return (
        start) -> {
      return (length) -> {
        return (buffer) -> {
          for (int i = start; i < (start + length); i++) {
            if (color == Color.kYellow) {
              // Custom yellow
              buffer.setRGB(i, 255, (int) (255 * 0.50), 0);
            } else {
              buffer.setLED(i, color);
            }
          }
          return buffer;
        };
      };
    };
  }

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbowChase = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double rainbowChaseSpeed = 100;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowChaseSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = (firstPixelHue + (i * 180 / length)) % 180;
          buffer.setHSV(i, hue, 255, 128);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbowBreatheSlow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double rainbowBreathSpeedSlow = 50;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowBreathSpeedSlow) % 180);
        for (int i = start; i < (start + length); i++) {
          buffer.setHSV(i, firstPixelHue, 255, 128);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbowBreatheFast = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double rainbowBreathSpeedFast = 500;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowBreathSpeedFast) % 180);
        for (int i = start; i < (start + length); i++) {
          buffer.setHSV(i, firstPixelHue, 255, 128);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redChase = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double chaseSpeed = 100;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * chaseSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = Helpers.clamp((firstPixelHue + (i * 180 / length)) % 180, 10, 255);
          buffer.setRGB(i, hue, 0, 0);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redBreathe = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double breatheSpeed = 205;

        int r = (int) (Math.pow(Math.sin(System.currentTimeMillis() / 1000.0), 2) * breatheSpeed) + 50;
        for (int i = start; i < (start + length); i++) {
          buffer.setRGB(i, r, 0, 0);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redTwinkleSlow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double twinkleRate = 0.001;
        int reduction = 5;

        for (int i = start; i < (start + length); i++) {
          // Get the LED at the current index
          Color led = buffer.getLED(i);
          int r = (int) (led.red * 255);

          // If its red component is greater than 0, then we want to decrease it
          if (r > 0 && Math.random() < 1.0) {
            r -= reduction;
          } else {
            // Otherwise, at a random chance, set it to full red
            if (Math.random() < twinkleRate) {
              r = 255;
            }
          }
          r = Helpers.clamp(r, 0, 255);
          buffer.setRGB(i, r, 0, 0);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redTwinkleFast = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double twinkleRate = 0.01;
        int reduction = 1;

        for (int i = start; i < (start + length); i++) {
          // Get the LED at the current index
          Color led = buffer.getLED(i);
          int r = (int) (led.red * 255);

          // If its red component is greater than 0, then we want to decrease it
          if (r > 0 && Math.random() < 1.0) {
            r -= reduction;
          } else {
            // Otherwise, at a random chance, set it to full red
            if (Math.random() < twinkleRate) {
              r = 255;
            }
          }
          r = Helpers.clamp(r, 0, 255);
          buffer.setRGB(i, r, 0, 0);
        }
        return buffer;
      };
    };
  };

  // TODO: Add mode that takes in a color and sets all the leds to that color
}
