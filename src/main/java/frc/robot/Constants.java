package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmPose;

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
    public static final double k_yDistance = 0.6858; // in meters! Side-to-Side

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
      public static final double k_FLOffset = 0.129074;
      public static final double k_FROffset = 0.722532;
      public static final double k_BLOffset = 0.205462;
      public static final double k_BROffset = 0.412673;

      public static final int k_FLMotorId = 9;
      public static final int k_FRMotorId = 10;
      public static final int k_BLMotorId = 11;
      public static final int k_BRMotorId = 12;
    }
  }

  public static class Arm {
    // TODO Update for actual robot
    public static final double k_shoulderPivotHeight = 19; // Inches
    public static final double k_homeHeight = Constants.Arm.k_shoulderPivotHeight + Constants.Arm.Shoulder.k_length
        - Constants.Arm.Elbow.k_length;

    public static class Shoulder {
      public static final int k_motorId = 13;
      public static final int k_encoderId = 2; // DIO
      public static final double k_length = 30.75; // Inches
      public static final double k_mass = Units.lbsToKilograms(15); // Kg
      public static final double k_minAngle = Units.degreesToRadians(-45.0);
      public static final double k_maxAngle = Units.degreesToRadians(225.0);

      public static final double k_offset = 0.0;
    }

    public static class Elbow {
      public static final int k_motorId = 14;
      public static final int k_encoderId = 1; // DIO
      public static final double k_length = 38.25; // Inches
      public static final double k_mass = Units.lbsToKilograms(100); // Kg
      public static final double k_minAngle = Units.degreesToRadians(-360.0);
      public static final double k_maxAngle = Units.degreesToRadians(360.0);

      public static final double k_offset = 0.124901;
    }

    public static class Wrist {
      public static final int k_motorId = 15;
      public static final int k_encoderId = 0; // DIO
      public static final double k_length = 1; // Inches TODO: May remove
      public static final double k_mass = Units.lbsToKilograms(5); // Kg
      public static final double k_minAngle = Units.degreesToRadians(-360.0);
      public static final double k_maxAngle = Units.degreesToRadians(360.0);

      public static final double k_offset = 0.162329;
    }

    public static enum Preset {
      HOME(new ArmPose(0.0, Constants.Arm.k_homeHeight, null)),
      SCORE_MID_CUBE(new ArmPose(0.0, 0.0, null)),
      SCORE_HIGH_CUBE(new ArmPose(0.0, 0.0, null)),
      SCORE_MID_CONE(new ArmPose(0.0, 0.0, null)),
      SCORE_HIGH_CONE(new ArmPose(52.5, 63.5, null)),
      SINGLE_SUBSTATION(new ArmPose(0.0, 0.0, null)),
      DOUBLE_SUBSTATION(new ArmPose(0.0, 0.0, null)),
      FLOOR_CONE(new ArmPose(26.0, 3.5, null)); //19.5, 2.0

      private ArmPose m_armPose;

      private Preset(ArmPose pose) {
        this.m_armPose = pose;
      }

      public ArmPose getPose() {
        return this.m_armPose;
      }
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
