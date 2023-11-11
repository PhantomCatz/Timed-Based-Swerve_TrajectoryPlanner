package frc.Autonomous.Actions;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.Odometry.CatzRobotTracker;
import frc.Mechanisms.drivetrain.CatzDrivetrain;
import frc.Utils.GeometryUtils;
import frc.robot.CatzConstants;
import frc.robot.Robot;

// Follows a trajectory
public class TrajectoryFollowingAction implements ActionBase{
    private final double TIMEOUT_RATIO = 1.5;
    private final double END_POS_ERROR = 0.05;
    private final double END_ROT_ERROR = 5;

    private final Timer timer = new Timer();
    private final HolonomicDriveController controller;
    private final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();

    private final Trajectory trajectory;
    private final Rotation2d targetHeading;
    private Rotation2d initHeading;

    CatzLog data;

    /**
     * @param trajectory The trajectory to follow
     * @param refHeading The goal heading for the robot to be in while in the middle of the trajectory. Takes a Pose2d parameter so that the heading may change based on external factors. 
     */
    public TrajectoryFollowingAction(Trajectory trajectory, Rotation2d targetHeading)
    {
        this.trajectory = trajectory;
        this.targetHeading = targetHeading; // this returns the desired orientation when given the current position (the function itself is given as an argument). But most of the times, it will just give a constant desired orientation.
        // also, why is it called refheading? wouldn't something like targetOrientation be better

        controller = CatzConstants.DriveConstants.holonomicDriveController; // see catzconstants
    }

    // reset and start timer
    @Override
    public void init() {
        timer.reset();
        timer.start();
        initHeading = driveTrain.getRotation2d();
    }

    // calculates if trajectory is finished
    @Override
    public boolean isFinished() {
        double maxTime = trajectory.getTotalTimeSeconds();
        Pose2d currentPosition = robotTracker.getEstimatedPosition();
        Pose2d dist = trajectory.sample(maxTime).poseMeters.relativeTo(currentPosition);

        return 
            timer.get() > maxTime * TIMEOUT_RATIO || 
            (
                Math.abs(targetHeading.getDegrees() - currentPosition.getRotation().getDegrees()) <= END_ROT_ERROR &&
                Math.hypot(dist.getX(), dist.getY()) <= END_POS_ERROR
            );
    }

    // sets swerve modules to their target states so that the robot will follow the trajectory
    // see catzconstants
    @Override
    public void update() {
        double currentTime = timer.get();
        Trajectory.State goal = trajectory.sample(currentTime);
        Pose2d currentPosition = robotTracker.getEstimatedPosition();
        Rotation2d targetHeadingNow = initHeading.interpolate(targetHeading, currentTime / trajectory.getTotalTimeSeconds());
        
        ChassisSpeeds adjustedSpeed = controller.calculate(currentPosition, goal, targetHeadingNow);
        //adjustedSpeed = correctForDynamics(adjustedSpeed);
        SwerveModuleState[] targetModuleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
        driveTrain.setSwerveModuleStates(targetModuleStates);

        if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_TRAJECTORY)){
            data = new CatzLog( 
                currentTime, 
                currentPosition.getX(), currentPosition.getY(), currentPosition.getRotation().getDegrees(),
                goal.poseMeters.getX(), goal.poseMeters.getY(), targetHeadingNow.getDegrees(),
                adjustedSpeed.vxMetersPerSecond, adjustedSpeed.vyMetersPerSecond, adjustedSpeed.omegaRadiansPerSecond,
                targetModuleStates[0].speedMetersPerSecond,
                driveTrain.LT_FRNT_MODULE.getModuleState().speedMetersPerSecond,
                targetModuleStates[0].angle.getDegrees(),
                driveTrain.LT_FRNT_MODULE.getModuleState().angle.getDegrees(),
                0.0, 0
            );                 
            Robot.dataCollection.logData.add(data);
        }

        /*Logger.getInstance().recordOutput("Current Position", robotTracker.getEstimatedPosition());
        Logger.getInstance().recordOutput("Target Position", goal.poseMeters);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelW", adjustedSpeed.omegaRadiansPerSecond);*/
    }

    // stop all robot motion
    @Override
    public void end() {
        timer.stop();

        driveTrain.stopDriving();
        
        System.out.println("trajectory done");
    }

            /**
     * Correction for swerve second order dynamics issue. Borrowed from 254:
     * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
     * Discussion:
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) 
    {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }
}