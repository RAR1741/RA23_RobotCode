package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Drivetrain {
    // Drivetrain wheel offsets
    // TODO: Change for final robot
    public static final double kXDistance = 0.762; // 30 inches
    public static final double kYDistance = 0.762; // in meters!

    public static final double kXCenterDistance = kXDistance / 2.0;
    public static final double kYCenterDistance = kYDistance / 2.0;

    // Drivetrain drive motor constants
    public static class Drive {
      public static final int kFLDriveMotorId = 5;
      public static final int kFRDriveMotorId = 6;
      public static final int kBLDriveMotorId = 7;
      public static final int kBRDriveMotorId = 8;
    }

    // Drivetrain (turn) constants
    public static class Turn {
      // Drivetrain turning offset constants
      public static final double kFLTurnOffset = 0.130769;
      public static final double kFRTurnOffset = 0.720806;
      public static final double kBLTurnOffset = 0.198625;
      public static final double kBRTurnOffset = 0.425397;

      public static final int kFLTurnMotorId = 9;
      public static final int kFRTurnMotorId = 10;
      public static final int kBLTurnMotorId = 11;
      public static final int kBRTurnMotorId = 12;
    }
  }

  public static class Arm {
    public static class Shoulder {
      public static final int kMotorId = 13;
      public static final double kMinAngle = Units.degreesToRadians(-45.0);
      public static final double kMaxAngle = Units.degreesToRadians(45.0);
    }

    public static class Elbow {
      public static final int kMotorId = 14;
      public static final double kMinAngle = Units.degreesToRadians(-45.0);
      public static final double kMaxAngle = Units.degreesToRadians(45.0);
    }
  }
}
