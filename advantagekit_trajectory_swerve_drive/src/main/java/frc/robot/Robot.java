// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.CatzRobotTracker;
import frc.robot.subsystems.CatzDriveTrainSubsystem;
import frc.robot.Autonomous.CatzAutonomous;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public static AHRS navX;
  private RobotContainer m_robotContainer;

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> sideChooser = new SendableChooser<>();

  private final CatzAutonomous auton = CatzAutonomous.getInstance();
  private final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private final CatzDriveTrainSubsystem drivetrain = CatzDriveTrainSubsystem.getInstance();

  private static final Timer autonTimer = new Timer();

  @Override
  public void robotInit() 
  {
    Logger logger = Logger.getInstance();


    

    // Set up data receivers & replay source
    switch (CatzConstants.currentMode) 
    {
      // Running on a real robot, log to a USB stick
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        logger.addDataReceiver(new WPILOGWriter("C:/Users/Ky Nam Le Nghiem/Desktop/"));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    m_robotContainer = new RobotContainer();

    AutonomousContainer.getInstance().initialize(
      true,
      new CommandTranslator(
              auton::setAutoPath,
              auton::stopMovement,
              auton::setAutoRotation,
              auton::isFinished,
              auton::getAutoElapsedTime,
              robotTracker::resetPosition,
              true
      ),
      false,
      null,
      this
    );

    AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

    sideChooser.setDefaultOption("Blue", "blue");
    sideChooser.addOption("Red", "red");

    SmartDashboard.putData("Path Choices", autoChooser);
    SmartDashboard.putData("Red or Blue", sideChooser);

    navX = new AHRS();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() 
  {
    autonTimer.reset();
    autonTimer.start();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static Timer getAutonTimer()
  {
    return autonTimer;
  }

 // public static AHRS getNavXObject() 
  {
   // return navX;
  }
}

