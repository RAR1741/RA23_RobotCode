package frc.robot;

public final class Constants {
  public class Drivetrain {
    // Drivetrain wheel offsets
    // TODO: Change for final robot
    public static final double kXDistance = 0.762; // 30 inches
    public static final double kYDistance = 0.762; // in meters!

    public static final double kXCenterDistance = kXDistance / 2.0;
    public static final double kYCenterDistance = kYDistance / 2.0;

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    // Drivetrain drive motor constants
    public class Drive {
      public static final int kFLMotorId = 5;
      public static final int kFRMotorId = 6;
      public static final int kBLMotorId = 7;
      public static final int kBRMotorId = 8;
    }

    // Drivetrain (turn) constants
    public class Turn {
      // Drivetrain turning offset constants
      public static final double kFLOffset = 0.130769;
      public static final double kFROffset = 0.720806;
      public static final double kBLOffset = 0.198625;
      public static final double kBROffset = 0.425397;

      public static final int kFLMotorId = 9;
      public static final int kFRMotorId = 10;
      public static final int kBLMotorId = 11;
      public static final int kBRMotorId = 12;
    }
  }
}
