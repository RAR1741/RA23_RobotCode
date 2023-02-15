package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Robot {
    public static final double kWidth = 27; // Inches
    public static final double kLength = 30; // Inches

    public static final double kBumperStart = 1; // Inches
    public static final double kBumperHeight = 5; // Inches
  }

  public static class Simulation {
    public static final double kWidth = 150; // Inches
    public static final double kHeight = 80; // Inches
  }

  public static class Drivetrain {
    // Drivetrain wheel offsets
    // TODO: Change for final robot
    public static final double k_xDistance = 0.762; // 30 inches
    public static final double k_yDistance = 0.762; // in meters!

    public static final double k_xCenterDistance = k_xDistance / 2.0;
    public static final double k_yCenterDistance = k_yDistance / 2.0;

    public static final double k_maxSpeed = 3.0; // 3 meters per second
    public static final double k_maxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double k_slowScaler = 0.2;

    // Drivetrain drive motor constants
    public class Drive {
      public static final int k_FLMotorId = 5;
      public static final int k_FRMotorId = 6;
      public static final int k_BLMotorId = 7;
      public static final int k_BRMotorId = 8;
    }

    // Drivetrain (turn) constants
    public static class Turn {
      // Drivetrain turning offset constants
      public static final double k_FLOffset = 0.130769;
      public static final double k_FROffset = 0.720806;
      public static final double k_BLOffset = 0.198625;
      public static final double k_BROffset = 0.425397;

      public static final int k_FLMotorId = 9;
      public static final int k_FRMotorId = 10;
      public static final int k_BLMotorId = 11;
      public static final int k_BRMotorId = 12;
    }
  }

  public static class Arm {
    public static final double kShoulderPivotHeight = 18; // Inches

    public static class Shoulder {
      public static final int kMotorId = 13;
      public static final double kLength = 30; // Inches
      public static final double kMass = Units.lbsToKilograms(15); // Kg
      public static final double kMinAngle = Units.degreesToRadians(-45.0);
      public static final double kMaxAngle = Units.degreesToRadians(225.0);
    }

    public static class Elbow {
      public static final int kMotorId = 14;
      public static final double kLength = 30; // Inches
      public static final double kMass = Units.lbsToKilograms(100); // Kg
      public static final double kMinAngle = Units.degreesToRadians(-360.0);
      public static final double kMaxAngle = Units.degreesToRadians(360.0);
    }
  }

  public static class Field {
    // All dimensions from Figure 5-16 in the manual
    public static final double kLowGoalX = 22.75; // Inches
    public static final double kLowGoalHeight = 34; // Inches

    public static final double kHighGoalX = 39.75; // Inches
    public static final double kHighGoalHeight = 46; // Inches
  }
}
