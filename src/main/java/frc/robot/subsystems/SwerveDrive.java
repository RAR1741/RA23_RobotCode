package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;

public class SwerveDrive extends Subsystem {

    private static SwerveDrive swerve = null;

    private final TalonFX m_driveFL;
    private final TalonFX m_driveFR;
    private final TalonFX m_driveBL;
    private final TalonFX m_driveBR;
    private final CANSparkMax m_rotFL;
    private final CANSparkMax m_rotFR;
    private final CANSparkMax m_rotBL;
    private final CANSparkMax m_rotBR;
    private final AHRS m_navX;

    private final Translation2d m_fL;
    private final Translation2d m_fR;
    private final Translation2d m_bL;
    private final Translation2d m_bR;
    private final SwerveDriveKinematics m_kinematics;

    public static SwerveDrive getInstance() {
        if(swerve == null) {
            swerve = new SwerveDrive(Constants.k_talFL, Constants.k_talFR, Constants.k_talBL, Constants.k_talBR, Constants.k_neoFL, Constants.k_neoFR,
            Constants.k_neoBL, Constants.k_neoBR);
        }
        return swerve;
    }

    private SwerveDrive(int tal_fL, int tal_fR, int tal_bL, int tal_bR, int neo_fL, int neo_fR, int neo_bL, int neo_bR) {
        m_driveFL = new TalonFX(tal_fL);
        m_driveFR = new TalonFX(tal_fR);
        m_driveBL = new TalonFX(tal_bL);
        m_driveBR = new TalonFX(tal_bR);

        m_rotFL = new CANSparkMax(neo_fL, MotorType.kBrushless);
        m_rotFR = new CANSparkMax(neo_fR, MotorType.kBrushless);
        m_rotBL = new CANSparkMax(neo_bL, MotorType.kBrushless);
        m_rotBR = new CANSparkMax(neo_bR, MotorType.kBrushless);

        m_navX = new AHRS(SPI.Port.kMXP);

        m_fL = new Translation2d(Constants.k_xDistance, Constants.k_yDistance);
        m_fR = new Translation2d(Constants.k_xDistance, -Constants.k_yDistance);
        m_bL = new Translation2d(-Constants.k_xDistance, Constants.k_yDistance);
        m_bR = new Translation2d(-Constants.k_xDistance, -Constants.k_yDistance);
        m_kinematics = new SwerveDriveKinematics(m_fL, m_fR, m_bL, m_bR);
    }

    /**
     * 
     * @param x = m/s forward (+) or backward (-)
     * @param y = m/s left (+) or right (-)
     * @param z = rad/sec counterclockwise (+) or clockwise (-)
     */
    public void swerveDrive(double x, double y, double z) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-x, -y, (z * 360) * Math.PI / 180, m_navX.getRotation2d());

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState m_fL = moduleStates[0];
        
        // Front right module state
        SwerveModuleState m_fR = moduleStates[1];
        
        // Back left module state
        SwerveModuleState m_bL = moduleStates[2];
        
        // Back right module state
        SwerveModuleState m_bR = moduleStates[3];

        var frontLeftOptimized = SwerveModuleState.optimize(m_fL, new Rotation2d(m_rotFL.getEncoder().getPosition()));

    }

    @Override
    public void periodic() {
        
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