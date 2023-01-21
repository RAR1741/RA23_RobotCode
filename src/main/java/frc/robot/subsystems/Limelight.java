package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable limelight;

    private boolean hasValidTargets;

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void tick() {

    }
}
