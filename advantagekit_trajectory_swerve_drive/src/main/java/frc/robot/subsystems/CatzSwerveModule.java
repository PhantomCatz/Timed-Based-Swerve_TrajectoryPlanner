/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.CatzConstants;


public class CatzSwerveModule
{
    private final CANSparkMax STEER_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private final int MOTOR_ID;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    private PIDController steeringPID;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double currentAngle = 0.0;
    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    public boolean driveDirectionFlipped = false;

    private final double WHEEL_OFFSET;

    public static final SendableChooser<Boolean> chosenState = new SendableChooser<>();

    //current limiting
    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int     STEER_CURRENT_LIMIT_AMPS      = 30;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset)
    {
        STEER_MOTOR = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorID);

        STEER_MOTOR.restoreFactoryDefaults();
        DRIVE_MOTOR.configFactoryDefault();

        //Set current limit
        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        STEER_MOTOR.setSmartCurrentLimit(STEER_CURRENT_LIMIT_AMPS);
        DRIVE_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);

        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        
        MagEncPWMInput = new DigitalInput(encoderDIOChannel);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        steeringPID = new PIDController(kP, kI, kD);

        this.WHEEL_OFFSET = offset;
        //for shuffleboard
        this.MOTOR_ID = steerMotorID;
        
    }

    public void resetMagEnc()
    {
        magEnc.reset();
    }

    public void setBrakeMode()
    {
        STEER_MOTOR.setIdleMode(IdleMode.kBrake);

    }
    public void setCoastMode()
    {
        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
    }

    public double closestAngle(double startAngle, double targetAngle)
    {
        // get direction
        double error = targetAngle % 360.0 - startAngle % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(error) > 180.0)
        {
            error = -(Math.signum(error) * 360.0) + error;
            //closest angle shouldn't be more than 180 degrees. If it is, use other direction
            if(error > 180.0)
            {
                error -= 360;
            }
        }

        return error;
    }

    public void setWheelAngle(double target, double gyroAngle)
    {
        currentAngle = ((magEnc.get() - WHEEL_OFFSET) * 360.0) - gyroAngle;
        // find closest angle to target angle
        angleError = closestAngle(currentAngle, target);
        
        // find closest angle to target angle + 180
        flippedAngleError = closestAngle(currentAngle, target + 180.0);

        // if the closest angle to target is shorter
        if (Math.abs(angleError) <= Math.abs(flippedAngleError))
        {
            driveDirectionFlipped = false;
            command = steeringPID.calculate(currentAngle, currentAngle + angleError);
        }
        // if the closest angle to target + 180 is shorter
        else
        {
            driveDirectionFlipped = true;
            command = steeringPID.calculate(currentAngle, currentAngle + flippedAngleError);
        }

        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        STEER_MOTOR.set(command);
    }

    public void setDesiredState(SwerveModuleState desiredState)
    {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentRotation()); //optimizes wheel rotation so that the furthest a wheel will ever rotate is 90 degrees.

        double percentOutput = desiredState.speedMetersPerSecond / CatzConstants.MAX_SPEED;
        
        DRIVE_MOTOR.set(ControlMode.PercentOutput, percentOutput);

        double targetAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (CatzConstants.MAX_SPEED * 0.01)) ? getCurrentRotation().getDegrees() : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angleError = CatzMathUtils.closestAngle(getCurrentRotation().getDegrees(), targetAngle);

        double command = steeringPID.calculate(getCurrentRotation().getDegrees(), angleError);
        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        STEER_MOTOR.set(command);
    }

    public void setSteerPower(double pwr)
    {
        STEER_MOTOR.set(pwr);
    }

    public void setDrivePower(double pwr)
    {
        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
        DRIVE_MOTOR.set(ControlMode.PercentOutput, -pwr);
    }
    
    public double getEncValue()
    {
        return magEnc.get();
    }

    public double getDrvDistanceRaw()
    {
        return DRIVE_MOTOR.getSelectedSensorPosition();
    }

    public double getDrvDistance()
    {
        if(driveDirectionFlipped)
        {
            return DRIVE_MOTOR.getSelectedSensorPosition();
        }
        else
        {
            return -DRIVE_MOTOR.getSelectedSensorPosition();
        }
    }

    public void resetDrvDistance()
    {
        int i = 0;

        DRIVE_MOTOR.setSelectedSensorPosition(0.0);
        while(Math.abs(DRIVE_MOTOR.getSelectedSensorPosition()) > 1.0)
        {
            i++;
            if(i >= 3000)
            {
                resetDrvDistance();
            }
        }
    }

    public double getDrvVelocity()
    {
        return DRIVE_MOTOR.getSelectedSensorVelocity();
    }
    
    public double getAngle()
    {
        return magEnc.get();//currentAngle
    }

    public double getError()
    {
        return angleError;
    }

    public double getFlipError()
    {
        return flippedAngleError;
    }

    public void smartDashboardModules()
    {
        SmartDashboard.putNumber(MOTOR_ID + " Wheel Angle", (currentAngle));
    }

    public void smartDashboardModules_DEBUG()
    {
        SmartDashboard.putNumber(MOTOR_ID + " Mag Encoder", magEnc.get() );
        //SmartDashboard.putBoolean(motorID + " Flipped", driveDirectionFlipped);
    }

    /*Auto Balance */
    public void reverseDrive(Boolean reverse)
    {
        DRIVE_MOTOR.setInverted(reverse);
    }

    public void resetDriveEncs()
    {
        DRIVE_MOTOR.setSelectedSensorPosition(0);
    }

    public void initializeOffset()
    {
    //    wheelOffset = magEnc.get();
    }

    private Rotation2d getCurrentRotation()
    {
        return Rotation2d.fromDegrees(magEnc.get() - WHEEL_OFFSET);
    }

    public SwerveModuleState getModuleState()
    {
        double velocity = CatzMathUtils.velocityCntsToMPS(DRIVE_MOTOR.getSelectedSensorVelocity(),CatzConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.SDS_L1_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(Units.inchesToMeters(getDriveDistanceInch()), Rotation2d.fromDegrees(getCurrentRotation().getDegrees()));
    }

    public double getDriveDistanceInch()
    {
        return DRIVE_MOTOR.getSelectedSensorPosition() * CatzConstants.SDS_L1_GEAR_RATIO * CatzConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }
}
