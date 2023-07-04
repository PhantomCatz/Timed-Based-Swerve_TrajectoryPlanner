// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzDriveTrainSubsystem;

public class DefaultDriveCommand extends CommandBase {

  private CatzDriveTrainSubsystem DRIVE_SUBSYSTEM;
  
  private final Supplier<Double> leftJoyXCmd, leftJoyYCmd, rightJoyXCmd, navXAngleCmd, pwrModeCmd;

  private boolean modifyDrvPwr = false;


  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(CatzDriveTrainSubsystem DRIVE_SUBSYSTEM, Supplier<Double> leftJoyXCmd, 
                                                            Supplier<Double> leftJoyYCmd, 
                                                            Supplier<Double> rightJoyXCmd, 
                                                            Supplier<Double> navXAngleCmd, 
                                                            Supplier<Double> pwrModeCmd   ) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = DRIVE_SUBSYSTEM;
    this.leftJoyXCmd     = leftJoyXCmd;
    this.leftJoyYCmd     = leftJoyYCmd;
    this.rightJoyXCmd    = rightJoyXCmd;
    this.navXAngleCmd    = navXAngleCmd;
    this.pwrModeCmd      = pwrModeCmd;

    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double realTimeLeftJoyX  = leftJoyXCmd.get();
    double realTimeLeftJoyY  = leftJoyYCmd.get();
    double realTimeRightJoyX = rightJoyXCmd.get();
    double realTimeNavXAngle = navXAngleCmd.get();
    double realtimePwrMode   = pwrModeCmd.get();

    double steerAngle = DRIVE_SUBSYSTEM.calcJoystickAngle(realTimeLeftJoyX, realTimeLeftJoyY);
    double drivePower = DRIVE_SUBSYSTEM.calcJoystickPower(realTimeLeftJoyX, realTimeLeftJoyY);
    double turnPower  = realTimeRightJoyX;
    
    double gyroAngle  = realTimeNavXAngle;

    double pwrMode = realtimePwrMode;

    

    if(pwrMode > 0.9)
    {
        modifyDrvPwr = true;
    }
    else
    {
        modifyDrvPwr = false;
    }

    if(drivePower >= 0.1)
    {
        
        if(modifyDrvPwr == true)
        {
            drivePower = drivePower * 0.5;
        }
        

        if(Math.abs(turnPower) >= 0.1)
        {
            if(modifyDrvPwr == true)
            {
                turnPower = turnPower * 0.5;

            }
            DRIVE_SUBSYSTEM.translateTurn(steerAngle, drivePower, turnPower, gyroAngle);
            }
           else
        {
          DRIVE_SUBSYSTEM.drive(steerAngle, drivePower, gyroAngle);
        }
    }
    else if(Math.abs(turnPower) >= 0.1)
    {
        if(modifyDrvPwr == true)
        {
            turnPower = turnPower * 0.5;
        }
        
        DRIVE_SUBSYSTEM.rotateInPlace(turnPower);
        
    }
    else
    {
      DRIVE_SUBSYSTEM.setSteerPower(0.0);
      DRIVE_SUBSYSTEM.setDrivePower(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
