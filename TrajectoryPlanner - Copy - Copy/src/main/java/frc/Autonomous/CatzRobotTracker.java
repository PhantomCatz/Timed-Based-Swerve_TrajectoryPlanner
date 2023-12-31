package frc.Autonomous;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.CatzDrivetrain;
import frc.robot.CatzConstants;

public class CatzRobotTracker extends AbstractMechanism{
    // combines SwerveDrivePoseEstimator with april tags to get robot position

    private static CatzRobotTracker instance = null;

    private static final int THREAD_PERIOD_MS = 20;

    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();
    private final CatzAprilTag limelight =  CatzAprilTag.getInstance();
    
    private SwerveDrivePoseEstimator poseEstimator;

    private CatzRobotTracker()
    {
        super(THREAD_PERIOD_MS);
        driveTrain.resetDriveEncs();
        poseEstimator = new SwerveDrivePoseEstimator(CatzConstants.swerveDriveKinematics, Rotation2d.fromDegrees(0), driveTrain.getModulePositions(), new Pose2d(0,0,Rotation2d.fromDegrees(0)));
        super.start();
    }

    //METHOD USED IN AUTONOMOUS CONTAINER
    public void resetPosition(Pose2d pose)
    {
        driveTrain.resetDriveEncs();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions(), pose);
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
        return poseEstimator.getEstimatedPosition();
    }

    // updates poseEstimator with new measurements
    @Override
    public void update() 
    {
        if(limelight.aprilTagInView())
        {
            //poseEstimator.addVisionMeasurement(limelight.getLimelightBotPose(), Timer.getFPGATimestamp());
        }
        poseEstimator.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions());
        System.out.println("Cur Pos: ("+poseEstimator.getEstimatedPosition().getX()+","+poseEstimator.getEstimatedPosition().getY()+")");
    }

    @Override
    public void smartDashboard() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void smartDashboard_DEBUG() {
        // TODO Auto-generated method stub
        
    }
}
