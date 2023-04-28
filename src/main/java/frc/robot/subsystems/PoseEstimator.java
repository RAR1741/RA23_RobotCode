package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class PoseEstimator extends Subsystem {

  private final Limelight m_limelight;
  private final SwerveDrive m_swerve;
  private final AprilTagFieldLayout m_layout;

  // TODO: Test with actual robot
  private static final Vector<N3> m_stateStandardDeviations = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  private static final Vector<N3> m_visionMeasurementStandardDeviations = VecBuilder.fill(0.5, 0.5,
      Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private double m_previousPipelineTimestamp = 0;

  private static PoseEstimator m_instance;

  public static PoseEstimator getInstance() {
    if (m_instance == null) {
      m_instance = new PoseEstimator();
    }
    return m_instance;
  }

  private PoseEstimator() {
    m_limelight = Limelight.getInstance();
    m_swerve = SwerveDrive.getInstance();

    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      Alliance alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
          : OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    m_layout = layout;

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_swerve.getKinematics(),
        m_swerve.getRotation2d(),
        m_swerve.getModulePositions(),
        new Pose2d(),
        m_stateStandardDeviations,
        m_visionMeasurementStandardDeviations);
  }

  @Override
  public void periodic() {
    LimelightResults limelightResult = LimelightHelpers.getLatestResults("limelight");
    double resultTimestamp = limelightResult.targetingResults.timestamp_LIMELIGHT_publish;
    if (resultTimestamp != m_previousPipelineTimestamp && m_limelight.seesAprilTag()) {
      m_previousPipelineTimestamp = resultTimestamp;
      // TODO: Make sure this is correct
      // (https://github.com/PhotonVision/photonvision/blob/6bdb158b332b87056748b6157c1aa8c3f5c77d8c/photon-lib/src/main/native/include/photonlib/PhotonPipelineResult.h#L63-L74)
      LimelightTarget_Fiducial target = limelightResult.targetingResults.targets_Fiducials[0];
      double fiducialId = target.fiducialID;
      // Get the tag pose from field layout - consider that the layout will be null if
      // it failed to load
      Optional<Pose3d> tagPose = m_layout == null ? Optional.empty()
          : m_layout.getTagPose((int) fiducialId);
      // if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 &&
      // tagPose.isPresent()) {
      // Pose3d targetPose = tagPose.get();
      // Transform3d camToTarget = target.getBestCameraToTarget();
      // Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
      // var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
      // m_poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(),
      // resultTimestamp);
      if (fiducialId >= 0 && tagPose.isPresent()) {
        m_poseEstimator.addVisionMeasurement(tagPose.get().toPose2d(), resultTimestamp);
      }
    }
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub

  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
