package frc.robot;

public final class Constants {
  public class Drivetrain {
    // Drivetrain wheel offsets
    // TODO: Change for final robot
    public static final double kXDistance = 0.762; // 30 inches
    public static final double kYDistance = 0.762; // in meters!

    public static final double kXCenterDistance = kXDistance/2.0;
    public static final double kYCenterDistance = kYDistance/2.0;

    // Drivetrain drive motor constants
    public class Drive {
      public static final int kFLDriveMotorId = 5;
      public static final int kFRDriveMotorId = 6;
      public static final int kBLDriveMotorId = 7;
      public static final int kBRDriveMotorId = 8;
    }

    // Drivetrain (turn) constants
    public class Turn {
      // Drivetrain turning offset constants
      public static final double kFLTurnOffset = 0.130769;
      public static final double kFRTurnOffset = 0.220806;
      public static final double kBLTurnOffset = 0.198625;
      public static final double kBRTurnOffset = 0.925397;

      public static final int kFLTurnMotorId = 9;
      public static final int kFRTurnMotorId = 10;
      public static final int kBLTurnMotorId = 11;
      public static final int kBRTurnMotorId = 12;

      public static final int kFLTurnEncoderChannelA = 0;
      public static final int kFLTurnEncoderChannelB = 1;

      public static final int kFRTurnEncoderChannelA = 4;
      public static final int kFRTurnEncoderChannelB = 5;

      public static final int kBLTurnEncoderChannelA = 2;
      public static final int kBLTurnEncoderChannelB = 3;

      public static final int kBRTurnEncoderChannelA = 6;
      public static final int kBRTurnEncoderChannelB = 7;
    }
  }
}
