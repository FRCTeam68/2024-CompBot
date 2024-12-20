// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.Camera;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;

  private final boolean UseLimelight = false;

  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers
    // uncomment line below to log to a USB stick, default if no parameters is: "/U/logs"
    Logger.addDataReceiver(new WPILOGWriter());
    // uncomment line below to log to Rio
    //Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));

    Logger.addDataReceiver(new NT4Publisher());

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // uncomment the line below to log CTRE devices to usb stick
    SignalLogger.setPath("//media/sda1/");
    // do not call the setPath and will be logged to rio at "/home/lvuser/logs"


    // SignalLogger.start();
    
    // Start AdvantageKit logger
    Logger.start();

    System.out.println("robot init");

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.m_DriveSubSystem.getDaqThread().setThreadPriority(99);

    System.out.println("CAN bus is FD: " + CANBus.isNetworkFD("DRIVEbus"));
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (UseLimelight) {    
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        m_robotContainer.m_DriveSubSystem.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
    }

    CANBusStatus canInfo = CANBus.getStatus("DRIVEbus");
    Logger.recordOutput("CANBUS/DRIVEbus/Util", canInfo.BusUtilization);
    Logger.recordOutput("CANBUS/DRIVEbus/Status",  canInfo.Status.getName());
    if (!canInfo.Status.isOK())
      Logger.recordOutput("CANBUS/DRIVEbus/Desc",  canInfo.Status.getDescription());
    
    CANBusStatus canInfo2 = CANBus.getStatus("rio");
    Logger.recordOutput("CANBUS/rio/Util", canInfo2.BusUtilization);
    Logger.recordOutput("CANBUS/rio/Status",  canInfo2.Status.getName());
    if (!canInfo2.Status.isOK())
      Logger.recordOutput("CANBUS/rio/Desc",  canInfo2.Status.getDescription());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.StopSubSystems();
    // SignalLogger.stop();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
