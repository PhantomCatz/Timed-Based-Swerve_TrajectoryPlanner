package frc.Mechanisms.drivetrain;


import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.Odometry.CatzAprilTag;
import frc.Mechanisms.Odometry.CatzRobotTracker;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.gameModeLED;

public class CatzDrivetrain {
    private static CatzDrivetrain instance = null;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public static CatzSwerveModule[] swerveModules = new CatzSwerveModule[4];
    private static CatzAprilTag aprilTag = CatzAprilTag.getInstance();

    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_BACK_DRIVE_ID = 22;
    private final int RT_FRNT_DRIVE_ID = 7;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_BACK_ENC_PORT = 7;
    private final int RT_FRNT_ENC_PORT = 8;

    /*private double LT_FRNT_OFFSET = 0.0091; 
    private double LT_BACK_OFFSET = 0.0466;
    private double RT_BACK_OFFSET = 0.2567;
    private double RT_FRNT_OFFSET = 0.0281;*/

    /*private double LT_FRNT_OFFSET = -0.2759; 
    private double LT_BACK_OFFSET = -0.1707;
    private double RT_BACK_OFFSET = 0.4169;
    private double RT_FRNT_OFFSET = 0.1975;*/

    private final double LT_FRNT_OFFSET =  0.0100; //0.073 //-0.0013; //MC ID 2
    private final double LT_BACK_OFFSET =  0.0439; //0.0431 //0.0498; //MC ID 4
    private final double RT_BACK_OFFSET =  0.2588; //0.2420 //0.2533; //MC ID 6
    private final double RT_FRNT_OFFSET =  0.0280; //0.0238 //0.0222; //MC ID 8

    private ChassisSpeeds chassisSpeeds;

    private CatzDrivetrain()
    {
        switch(CatzConstants.currentMode)
        {
        case REAL:
            gyroIO = new GyroIONavX();
        break;
        default:
            gyroIO = null; // TBD does gryo need sim class
        break;
        }

        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, 0);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, 1);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, 2);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, 3);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[1] = LT_BACK_MODULE;
        swerveModules[2] = RT_FRNT_MODULE;
        swerveModules[3] = RT_BACK_MODULE;

        resetMagEncs();
    }
    
    public void updateSensorValues(){
        for(CatzSwerveModule module : swerveModules)
        {
            module.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
    }

    public void cmdProcSwerve(double leftJoyX, double leftJoyY, double rightJoyX, boolean isAutoAlignCubeTrue)
    {
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);

        if(Math.sqrt(Math.pow(leftJoyX,2) + Math.pow(leftJoyY,2)) < 0.1){
            leftJoyX = 0.0;
            leftJoyY = 0.0;
        }
        
        if(isAutoAlignCubeTrue)
        {
            double autoAlignDrvXPwr;
            double autoAlignDrvYPwr;
            double autoAlignTurnPwr;
            if(aprilTag.disToTag() > 0.5){
                autoAlignDrvXPwr = 0.2;   
            }
            else if(aprilTag.disToTag() < 0.5){
                autoAlignDrvXPwr = -0.2;
            }
            else{
                autoAlignDrvXPwr = 0.0;
            }

            if(aprilTag.disLateralToTargetTag() > 0.1){
                autoAlignDrvYPwr = 0.2;
            }
            else if(aprilTag.disLateralToTargetTag() < -0.1)
            {
                autoAlignDrvYPwr = -0.2;
            }
            else{
                autoAlignDrvYPwr = 0.0;
            }

            
            if(aprilTag.angleErrorFromTag() > 0.1) {
                autoAlignTurnPwr = 0.2;
            }
            else if(aprilTag.angleErrorFromTag() < -0.1)
            {
                autoAlignTurnPwr = -0.2;
            }
            else
            {
                autoAlignTurnPwr = 0.0;
            }

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(autoAlignDrvXPwr * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  autoAlignDrvYPwr * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  autoAlignTurnPwr * CatzConstants.DriveConstants.MAX_ANGSPEED, 
                                                                  getRotation2d());
        }
        else //in full teleop
        {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(leftJoyX * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  leftJoyY * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  rightJoyX * CatzConstants.DriveConstants.MAX_ANGSPEED, 
                                                                  getRotation2d());
        }


     
        SwerveModuleState[] moduleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setSwerveModuleStates(moduleStates);
        

        Logger.getInstance().recordOutput("module states", moduleStates);

    }

    public void setSwerveModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, CatzConstants.DriveConstants.MAX_SPEED);
        for(int i = 0; i < states.length; i++){
            states[i] = SwerveModuleState.optimize(states[i], swerveModules[i].getCurrentRotation());
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getHeading() 
    {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    private void resetMagEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetMagEnc();
        }
    }

    public void resetDriveEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetDriveEncs();
        }
    }

    public void initializeOffsets()
    {
        //GyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);

        for(CatzSwerveModule module : swerveModules)
        {
            module.initializeOffset();
        }
    }

    public double getGyroAngle()
    {
        // negative sign makes positive counterclockwise, which is correct
        return - gyroInputs.gyroAngle;
    }

    public double getRoll(){
        return gyroInputs.gyroRoll;
    }

    public void stopDriving(){
        for(CatzSwerveModule module : swerveModules)
        {
            module.setPower(0.0);
            module.setSteeringPower(0.0);
        }
    }

    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        for(int i = 0; i < 4; i++)
        {
            moduleStates[i] = swerveModules[i].getModuleState();
        }

        return moduleStates;
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
        }

        return modulePositions;
    }

    public void printModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
            System.out.println(modulePositions[i].distanceMeters);
        }
    }

    public void zeroGyro()
    {
      gyroIO.resetNavXIO();
    }

    public void smartDashboardDriveTrain_DEBUG()
    {

    }

    public void smartDashboardDriveTrain()
    {
        SmartDashboard.putNumber("Gyro", getGyroAngle());
        SmartDashboard.putNumber("LF Angle", LT_FRNT_MODULE.getCurrentRotation().getDegrees());
        SmartDashboard.putNumber("LB Angle", LT_BACK_MODULE.getCurrentRotation().getDegrees());
        SmartDashboard.putNumber("RF Angle", RT_FRNT_MODULE.getCurrentRotation().getDegrees());
        SmartDashboard.putNumber("RB Angle", RT_BACK_MODULE.getCurrentRotation().getDegrees());
    }

    public static CatzDrivetrain getInstance()
    {
        if(instance == null)
        {
            instance = new CatzDrivetrain();
        }

        return instance;
    }
}