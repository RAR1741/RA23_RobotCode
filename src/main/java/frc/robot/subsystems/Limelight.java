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

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void tick() {
        hasValidTargets = limelight.getEntry("tv").getDouble(0) == 1;
        horizontalOffset = limelight.getEntry("tx").getDouble(0);
        verticalOffset = limelight.getEntry("ty").getDouble(0);
        targetArea = limelight.getEntry("ta").getDouble(0);

        SmartDashboard.putBoolean("HasValidTargets", hasValidTargets);
        SmartDashboard.putNumber("HorizontalOffset", horizontalOffset);
        SmartDashboard.putNumber("VerticalOffset", verticalOffset);
        SmartDashboard.putNumber("TargetArea", targetArea);
    }
}
