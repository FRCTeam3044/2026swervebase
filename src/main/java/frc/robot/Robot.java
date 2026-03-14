// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoTargetUtil;
import frc.robot.util.Elastic;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.ShotCalculator;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.PathfindingDebugUtils;
import me.nabdev.oxconfig.OxConfig;
import me.nabdev.pathfinding.structures.Vertex;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static RobotContainer robotContainer;

  public static Timer timer = new Timer();
  private Field2d field = new Field2d();

  private final Alert turretNotReset = new Alert("!!! TURRET NOT RESET !!!", AlertType.kError);
  private final Alert turretInDangerZone = new Alert("!!! TURRET IN DANGER ZONE !!!", AlertType.kError);
  private final Alert autoWinnerNotSet = new Alert("!!! AUTO WINNER NOT SET !!!", AlertType.kError);

  public Timer getTimer() {
    return timer;
  }

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    System.out.println("Constructing RobotContainer...");
    robotContainer = RobotContainer.getInstance();
    System.out.println("Initializing OxConfig...");

    OxConfig.initialize();
    System.out.println("Starting state machine...");
    robotContainer.stateMachine.onStartup();
    Logger.recordOutput("test path",
        DriveCommands.generateTrajectory(robotContainer.drive, new Pose2d(1, 1, new Rotation2d(0)),
            new Pose2d(2, 1, new Rotation2d(0))));

    // PathfindingDebugUtils.drawLines("Field Map",
    // DriveConstants.pathfinder.visualizeEdges(),
    // DriveConstants.pathfinder.visualizeVertices());

    // PathfindingDebugUtils.drawLines("Field Map Inflated",
    // DriveConstants.pathfinder.visualizeEdges(),
    // DriveConstants.pathfinder.visualizeInflatedVertices());

  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    robotContainer.stateMachine.periodic();

    SmartDashboard.putData(CommandScheduler.getInstance());
    // Logger.recordOutput("CommandScheduler", CommandScheduler.getInstance());

    SmartDashboard.putString("Alliance", AllianceUtil.getAlliance().toString());

    ShiftInfo official = HubShiftUtil.getOfficialShiftInfo();
    ShiftInfo shifted = HubShiftUtil.getShiftedShiftInfo();
    Logger.recordOutput("HubShift/Official", official);
    Logger.recordOutput("HubShift/Shifted", shifted);
    publishSchedule("ShiftedShift", shifted);
    publishSchedule("OfficialShift", official);
    field.setRobotPose(robotContainer.drive.getPose());
    SmartDashboard.putData(field);
    // SmartDashboard.putString(
    // "ShiftedShift/Text",
    // String.format("%.1f", Math.max(shifted.remainingTime(), 0.0)));

    Logger.recordOutput(
        "Distance from hub target",
        robotContainer.drive
            .getPose()
            .getTranslation()
            .getDistance(
                robotContainer.autoTargetUtil.getHub().getTranslation().toTranslation2d()));

    robotContainer.autoAim.periodic();

    turretNotReset.set(!robotContainer.turret.hasReset());
    turretInDangerZone.set(robotContainer.turret.inHoodDangerZone() && !robotContainer.hood.calibrated());

    ShotCalculator.periodic();
  }

  private void publishSchedule(String key, ShiftInfo info) {
    SmartDashboard.putBoolean(key + "/Boolean", info.active());
    SmartDashboard.putString(key + "/.type", "Status Display");
    SmartDashboard.putNumber(key + "/RemainingTime", Math.max(info.remainingTime(), -3.0));
    SmartDashboard.putNumber(key + "/ShiftLength", info.elapsedTime() + info.remainingTime());
    SmartDashboard.putString(key + "/ShiftName", info.currentShift().toString());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    HubShiftUtil.initialize();
    Elastic.selectTab(0);
    // robotContainer.turret.resetAngle(true);
  }

  public static int loopCount = 0;

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (loopCount < 15) {
      loopCount++;
    }
    AllianceUtil.setAlliance();
    robotContainer.turret.resetAngle(true);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    AllianceUtil.setAlliance();
    HubShiftUtil.initialize();
    Elastic.selectTab(1);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  private boolean autoWinnerAlerted = false;

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    AllianceUtil.setAlliance();
    HubShiftUtil.initialize();
    timer.restart();
    autoWinnerAlerted = false;
    Elastic.selectTab(1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (!(DriverStation.getGameSpecificMessage().length() > 0) &&
    // HubShiftUtil.getAllianceWinOverride().isEmpty()
    // && timer
    // .hasElapsed(1.0)) {
    // autoWinnerNotSet.set(true);
    // if (!autoWinnerAlerted) {
    // Elastic.sendNotification(
    // new Notification(NotificationLevel.ERROR, "AUTO WINNER NOT SET!!!!", "MERN
    // YOU NEED TO SET IT MERN", 30000,
    // 512, 256));
    // autoWinnerAlerted = true;
    // }
    // } else {
    // autoWinnerNotSet.set(false);
    // autoWinnerAlerted = false;
    // }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    AllianceUtil.setAlliance();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  double lastScore = 0;

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
    Logger.recordOutput(
        "FieldSimulation/Robot", RobotContainer.driveSimulation.getSimulatedDriveTrainPose());

    double currentScore = SmartDashboard.getNumber("MapleSim/MatchData/Breakdown/blue Alliance/TotalFuelInHub", 0);
    if (currentScore != lastScore) {
      lastScore = currentScore;
      SmartDashboard.putNumber("LastScoreTime", robotContainer.simShotTimer.get());
    }

    robotContainer.updateMechanism();
  }
}
