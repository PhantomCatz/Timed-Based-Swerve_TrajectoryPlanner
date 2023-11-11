package frc.Mechanisms.Odometry;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.drivetrain.CatzDrivetrain;
import frc.robot.CatzConstants;

public class CatzRobotTracker {
    // combines SwerveDrivePoseEstimator with april tags to get robot position

    private static CatzRobotTracker instance = null;

    private static final int THREAD_PERIOD_MS = 20;

    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();
    private final CatzAprilTag limelight =  CatzAprilTag.getInstance();
    
    private SwerveDrivePoseEstimator poseEstimator;

    private CatzRobotTracker()
    {
        driveTrain.resetDriveEncs();
        poseEstimator = new SwerveDrivePoseEstimator(
            CatzConstants.DriveConstants.swerveDriveKinematics, 
            CatzConstants.initPose.getRotation(), 
            driveTrain.getModulePositions(), 
            CatzConstants.initPose
        );
    }

    //METHOD USED IN AUTONOMOUS CONTAINER
    public void resetPosition(Pose2d pose)
    {
        driveTrain.resetDriveEncs();
        poseEstimator.resetPosition(CatzConstants.initPose.getRotation(), driveTrain.getModulePositions(), pose);
    }
    //METHOD USED IN AUTONOMOUS CONTAINER

    // returns itself
    public static CatzRobotTracker getInstance()
    {
        if(instance == null) 
        {
            instance = new CatzRobotTracker();
        }
        return instance;
    }

    // get position
    public Pose2d getEstimatedPosition()
    {
        Pose2d currentPosition = poseEstimator.getEstimatedPosition();
        currentPosition = new Pose2d(currentPosition.getX(), currentPosition.getY(), Rotation2d.fromDegrees(driveTrain.getGyroAngle()));
        return currentPosition;
    }

    // updates poseEstimator with new measurements
    public void update() 
    {
        if(limelight.aprilTagInView())
        {
            //poseEstimator.addVisionMeasurement(limelight.getLimelightBotPose(), Logger.getInstance().getRealTimestamp());
        }
        poseEstimator.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions());
        //System.out.println("("+poseEstimator.getEstimatedPosition().getX()+","+poseEstimator.getEstimatedPosition().getY()+")");
        Logger.getInstance().recordOutput("pose", poseEstimator.getEstimatedPosition());
    }

    public void smartDashboard() {
        // TODO Auto-generated method stub
        
    }

    public void smartDashboard_DEBUG() {
        // TODO Auto-generated method stub
        
    }
}
