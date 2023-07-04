// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.CatzDataLogger;
import frc.robot.commands.ManualCommands.DefaultDriveCommand;
import frc.robot.subsystems.CatzDriveTrainSubsystem;

public class RobotContainer 
{
   //datalogger
   public CatzDataLogger dataLogger;
   
   private final CatzConstants catzConstants;

   public static Timer             currentTime;

   //subsystems
   private final CatzDriveTrainSubsystem DRIVE_SUBSYSTEM;

   private CommandXboxController xboxDrv;
   private CommandXboxController xboxAux;

   //RobotContainer Constants
   private final int XBOX_DRV_PORT = 0;
   private final int XBOX_AUX_PORT = 1;

  public RobotContainer() 
  {
    currentTime = new Timer();
    dataLogger = new CatzDataLogger();
    catzConstants = new CatzConstants();


    DRIVE_SUBSYSTEM  = new CatzDriveTrainSubsystem();

    xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
    xboxAux = new CommandXboxController(XBOX_AUX_PORT);



    configureButtonBindings();
    defaultCommands();
  }

  private void configureButtonBindings() {}

  private void defaultCommands() 
  {
    DRIVE_SUBSYSTEM.setDefaultCommand(new DefaultDriveCommand(DRIVE_SUBSYSTEM,
                                                              () -> xboxDrv.getLeftX(),
                                                              () -> xboxDrv.getLeftY(), 
                                                              () -> xboxDrv.getRightX(), 
                                                              () -> Robot.navX.getAngle(), 
                                                              () -> xboxDrv.getRightTriggerAxis()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
