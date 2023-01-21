package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    private NetworkTable limelight;

    private boolean hasValidTargets;
    private double horizontalOffset;
    private double verticalOffset;
    private double targetArea;
    private double boundingBoxHorizontal;
    private double boundingBoxVertical;
    private int primaryAprilTagID;
    private Object botPos;

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void tick() {
        hasValidTargets = limelight.getEntry("tv").getDouble(0) == 1;
        horizontalOffset = limelight.getEntry("tx").getDouble(0);
        verticalOffset = limelight.getEntry("ty").getDouble(0);
        targetArea = limelight.getEntry("ta").getDouble(0);
        boundingBoxHorizontal = limelight.getEntry("thor").getDouble(0);
        boundingBoxVertical = limelight.getEntry("tvert").getDouble(0);
        primaryAprilTagID = (int) limelight.getEntry("tvert").getInteger(-1); // Uses -1 as default so when no AprilTags are found, we can use this alongside the valid target checker
        // botPos = limelight.getEntry("botpose").getClass(); // TODO: Find out return type(s) once the limelight is up and running
        
        SmartDashboard.putBoolean("HasValidTargets", hasValidTargets);
        SmartDashboard.putNumber("HorizontalOffset", horizontalOffset);
        SmartDashboard.putNumber("VerticalOffset", verticalOffset);
        SmartDashboard.putNumber("TargetArea", targetArea);
        SmartDashboard.putNumber("BoundingBoxVertical", boundingBoxVertical);
        SmartDashboard.putNumber("BoundingBoxHorizontal", boundingBoxHorizontal);
        SmartDashboard.putNumber("PrimaryAprilTagID", primaryAprilTagID);
    }
}
