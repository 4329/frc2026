// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import com.ctre.phoenix6.SignalLogger;
import frc.robot.utilities.HoorayConfig;
import java.io.File;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

  /**
   * Finds a suitable directory for logging data.
   */
  private File findThumbDir() {
      // First checks if /media contains any writable "logs" directories on flash drives
      File f = new File("/media");
      for (File kid : f.listFiles()) {
          File logs = new File(kid, "logs");
          if (logs.exists() && logs.canWrite()) {
              Logger.recordMetadata("Logging On:", "Flash Drive");
              return logs;
          } else if (logs.mkdir()) {
              Logger.recordMetadata("Logging On:", "Flash Drive");
              return logs;
          }
      }
      // If no flash drive found, logs to the robot's internal storage
      File homeDir = new File("/home/lvuser/logs");
      if (homeDir.exists() || homeDir.mkdir()) {
          Logger.recordMetadata("Logging On:", "Robot");
          return homeDir;
      } else {
          Logger.recordMetadata("Logging On:", "Nothing");
          return null;
      }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Set up Logger
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
        if (isReal()) {
            File logFolder = findThumbDir();
            if (logFolder != null) {
                Logger.addDataReceiver(
                        new WPILOGWriter(logFolder.getAbsolutePath())); // Log to a USB stick ("/U/logs")
            }
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            Constants.robotMode = Mode.REAL;
            SignalLogger.setPath(logFolder.getAbsolutePath());
        } else if (isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
            Constants.robotMode = Mode.SIM;

        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath =
                    LogFileUtil
                            .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(
                    new WPILOGWriter(
                            LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            Constants.robotMode = Mode.REPLAY;
        }

    // Logger setup complete, start logging
    SignalLogger.start();
    Logger.recordMetadata("mode", Constants.robotMode.toString());
    Logger.recordMetadata("encoderType", HoorayConfig.gimmeConfig().getEncoderType().toString());
    Logger.registerURCL(URCL.startExternal());
    Logger.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // Log robot periodic info
    m_robotContainer.robotPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Logger.recordOutput("Auto", m_robotContainer.getAutoName(m_autonomousCommand));
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
