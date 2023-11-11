package frc.Mechanisms.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.CatzConstants;

public class GyroIONavX implements GyroIO 
{
    private final AHRS navX;
  
    public GyroIONavX() {
        navX = new AHRS();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) 
    {
      inputs.gyroAngle = navX.getAngle();
      inputs.gyroYaw = navX.getYaw();
      inputs.gyroRoll = navX.getRoll();
    }

    @Override
    public void resetNavXIO()
    {
        navX.reset();
        navX.setAngleAdjustment(CatzConstants.initPose.getRotation().getDegrees());
    }

    @Override
    public void setAngleAdjustmentIO(double gyroYaw) 
    {
        navX.setAngleAdjustment(gyroYaw);
    }
}