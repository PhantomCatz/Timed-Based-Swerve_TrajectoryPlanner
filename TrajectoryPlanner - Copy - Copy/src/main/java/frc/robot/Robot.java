// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.AutonActionExecutor;
import frc.Autonomous.AutonRoutineSelector;
import frc.Autonomous.CatzRobotTracker;
import frc.Mechanisms.CatzDrivetrain;

@SuppressWarnings("unused")
public class Robot extends TimedRobot {
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> sideChooser = new SendableChooser<>();

  public static final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();
  private final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();

  public static final AutonActionExecutor autonExecutor = AutonActionExecutor.getInstance();
  private final AutonRoutineSelector autonRoutineSelector = AutonRoutineSelector.getInstance();

  private static final Timer autonTimer = new Timer();
  @Override
  public void robotInit()
  {
    // AutonomousContainer.getInstance().initialize(
    //             true,
    //             new CommandTranslator(
    //                     auton::setAutoPath,
    //                     auton::stopMovement,
    //                     auton::setAutoRotation,
    //                     auton::isFinished,
    //                     auton::getAutoElapsedTime,
    //                     robotTracker::resetPosition,
    //                     true
    //             ),
    //             false,
    //             this
    //     );

    //   AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

    //   sideChooser.setDefaultOption("Blue", "blue");
    //   sideChooser.addOption("Red", "red");

    //   SmartDashboard.putData("Path Choices", autoChooser);
    //   SmartDashboard.putData("Red or Blue", sideChooser);
    robotTracker.resetPosition(new Pose2d());
  }

  @Override 
  public void robotPeriodic(){
    SmartDashboard.putNumber("Left Back", drivetrain.LT_BACK_MODULE.getCurrentRotation().getDegrees());
    SmartDashboard.putNumber("Left Front", drivetrain.LT_FRNT_MODULE.getCurrentRotation().getDegrees());
    SmartDashboard.putNumber("Right Back", drivetrain.RT_BACK_MODULE.getCurrentRotation().getDegrees());
    SmartDashboard.putNumber("Right Front", drivetrain.RT_FRNT_MODULE.getCurrentRotation().getDegrees());
  }

  @Override
  public void autonomousInit()
  {
    // autonTimer.reset();
    // autonTimer.start();

    // AutonomousContainer.getInstance().runAutonomous(autoChooser.getSelected(), sideChooser.getSelected(), true);

    robotTracker.resetPosition(new Pose2d());
    autonRoutineSelector.updateSelectedRoutine();
    autonExecutor.start();    
  }

  @Override
  public void disabledInit()
  {
    autonTimer.stop();
    autonExecutor.stop();
    AutonActionExecutor.resetInstance();
  }

  @Override
  public  void testInit()
  {
    drivetrain.initializeOffsets();
  }

  public static Timer getAutonTimer()
  {
    return autonTimer;
  }
}
