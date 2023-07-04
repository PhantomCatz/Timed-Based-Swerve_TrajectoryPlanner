package frc.robot.Autonomous;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CatzDriveTrainSubsystem;

public class CatzRobotTracker extends SubsystemBase
{
    private static CatzRobotTracker instance = null;

    private static final int THREAD_PERIOD_MS = 20;

    private SwerveDrivePoseEstimator poseEstimator;

    private final CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
    private final CatzAprilTag limelight =  CatzAprilTag.getInstance();
    
    private CatzRobotTracker()
    {

    }

    //METHOD USED IN AUTONOMOUS CONTAINER
    public void resetPosition(Pose2d pose)
    {
        driveTrain.resetDriveEncs();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions(), pose);
    }
    //METHOD USED IN AUTONOMOUS CONTAINER

    public static CatzRobotTracker getInstance()
    {
        if(instance == null) 
        {
            instance = new CatzRobotTracker();
        }
        return instance;
    }

    public Pose2d getEstimatedPosition()
    {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() 
    {
        if(limelight.aprilTagInView())
        {
            poseEstimator.addVisionMeasurement(limelight.getLimelightBotPose(), Timer.getFPGATimestamp());
        }
        poseEstimator.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()),driveTrain.getModulePositions()); 
    }
}
