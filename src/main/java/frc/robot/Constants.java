package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Robot {
    public static final double k_width = 27; // Inches
    public static final double k_length = 30; // Inches

    public static final double k_bumperStart = 1; // Inches
    public static final double k_bumperHeight = 5; // Inches
  }

  public static class Simulation {
    public static final double k_width = 150; // Inches
    public static final double k_height = 80; // Inches
  }

  public class Drivetrain {
    // Drivetrain wheel offsets
    // TODO: Change for final robot
    public static final double k_xDistance = 0.762; // 30 inches Forward/Backward
    public static final double k_yDistance = 0.762; // in meters! Side-to-Side
    
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
    public class Turn {
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
    //TODO Update for actual robot
    public static final double k_shoulderPivotHeight = 18; // Inches

    public static class Shoulder {
      public static final int k_motorId = 13;
      public static final double k_length = 30; // Inches
      public static final double k_mass = Units.lbsToKilograms(15); // Kg
      public static final double k_minAngle = Units.degreesToRadians(-45.0);
      public static final double k_maxAngle = Units.degreesToRadians(225.0);

      public static final double k_defaultState = 0.000;
      public static final double k_lowState = 0.000;
      public static final double k_mediumState = 0.000;
      public static final double k_highState = 0.000;
      public static final double k_highPrimeState = 0.000;
      public static final double k_mediumPrimeState = 0.000;
      public static final double k_lowPrimeState = 0.000;
    }

    public static class Elbow {
      public static final int k_motorId = 14;
      public static final int k_encoderId = 1; // DIO
      public static final double k_length = 30; // Inches
      public static final double k_mass = Units.lbsToKilograms(100); // Kg
      public static final double k_minAngle = Units.degreesToRadians(-360.0);
      public static final double k_maxAngle = Units.degreesToRadians(360.0);

      public static final double k_defaultState = 0.000;
      public static final double k_lowState = 0.000;
      public static final double k_mediumState = 0.000;
      public static final double k_highState = 0.000;
      public static final double k_highPrimeState = 0.000;
      public static final double k_mediumPrimeState = 0.000;
      public static final double k_lowPrimeState = 0.000;
    }

    public static class Wrist {
      public static final int k_motorId = 14;
      public static final int k_encoderId = 0; // DIO
      public static final double k_length = 30; // Inches
      public static final double k_mass = Units.lbsToKilograms(5); // Kg
      public static final double k_minAngle = Units.degreesToRadians(-360.0);
      public static final double k_maxAngle = Units.degreesToRadians(360.0);

      public static final double k_defaultState = 0.000;
      public static final double k_lowState = 0.000;
      public static final double k_mediumState = 0.000;
      public static final double k_highState = 0.000;
      public static final double k_highPrimeState = 0.000;
      public static final double k_mediumPrimeState = 0.000;
      public static final double k_lowPrimeState = 0.000;
    }
  }

  public static class Field {
    // All dimensions from Figure 5-16 in the manual
    public static final double k_lowGoalX = 22.75; // Inches
    public static final double k_lowGoalHeight = 34; // Inches

    public static final double k_highGoalX = 39.75; // Inches
    public static final double k_highGoalHeight = 46; // Inches
  }
}
