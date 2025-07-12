// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.VisionSystem;
import frc.utilities.PosUtils;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private String autoName, newAutoName;
  private Optional<Alliance> alliance, newAlliance;
  @Logged
  public final Field2d m_field = new Field2d();

  @Logged
  private final RobotContainer m_robotContainer;
  public static boolean isFMS = false;
  public static boolean isEStop = false;
  public static boolean isBatteryLow = false;
  public static int musicTimer = 0;

  // creates an orchestra, for playing .chrp music files over our motors
  public static Orchestra jukebox = new Orchestra();
  // creates audio config, for enabling play when disabled
  private AudioConfigs soundSettings = new AudioConfigs();

  static void startServer() {
    System.out.println("I AM STARTING WEBSERVER =======================================================");
    String path = Filesystem.getDeployDirectory().getPath() + "\\ElasticLayout";
    WebServer.start(5800, path);
    // PortForwarder.add(5800, "localhost", 5900);
  }

  public Robot() {
    DataLogManager.start("/media/sda1/logs/RIO");
    Epilogue.configure(config -> {
      config.backend = new FileBackend(DataLogManager.getLog());
      config.errorHandler = ErrorHandler.printErrorMessages();
    });
    Epilogue.bind(this);
    startServer();
    m_robotContainer = new RobotContainer();
    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand().schedule();
    SmartDashboard.putData("Field", m_field);

    soundSettings.AllowMusicDurDisable = true;
    // adds all of the swerve pod motors as instruments for playing music via the
    // orchestra (jukebox)
    for (int i = 0; i < 4; i++) {
      jukebox.addInstrument(m_robotContainer.drivetrain.getModule(i).getDriveMotor(), 0);
      jukebox.addInstrument(m_robotContainer.drivetrain.getModule(i).getSteerMotor(), 1);
    }
    // adds the thing to the jukebox
    jukebox.loadMusic("BuddyHollyRiff.chrp");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateVisionPose(false);
    m_field.setRobotPose(m_robotContainer.visionSystem.getEstimatedGlobalPose2d());

    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

    if (!isFMS && DriverStation.isFMSAttached()) {
      isFMS = true;
      RobotContainer.prettyLights.isFMS();
    }

    if (!isEStop && DriverStation.isEStopped()) {
      isEStop = true;
      RobotContainer.prettyLights.isEStop();
      // for patrick
      if (!isFMS) {
        jukebox.pause();
        jukebox.loadMusic("blackParade.chrp");
        jukebox.play();
      }

    }

    if (RobotController.getBatteryVoltage() < LEDConstants.BATTERY_WARNING_VOLTAGE) {
      isBatteryLow = true;
    } else {
      isBatteryLow = false;
    }
    // this one is outside so that the pattern can be removed once the voltage goes
    // back up
    RobotContainer.prettyLights.batteryLow(isBatteryLow);

  }

  @Override
  public void disabledInit() {
    m_robotContainer.updateVisionPose(true);
  }

  @Override
  public void disabledPeriodic() {
    m_field.setRobotPose(m_robotContainer.visionSystem.getEstimatedGlobalPose2d());

    newAlliance = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName || alliance != newAlliance) {
      autoName = newAutoName;
      alliance = newAlliance;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        try {
          List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          List<Pose2d> poses = new ArrayList<>();
          for (PathPlannerPath path : pathPlannerPaths) {
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
              path = path.flipPath();
            }
            poses.addAll(path.getAllPathPoints().stream()
                .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                .collect(Collectors.toList()));
          }
          m_field.getObject("path").setPoses(poses);
        } catch (IOException | org.json.simple.parser.ParseException e) {
          e.printStackTrace();
        }
      }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    PosUtils.selectTab(1);
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.updateVisionPose(false);
    m_field.setRobotPose(m_robotContainer.visionSystem.getEstimatedGlobalPose2d());
  }

  @Override
  public void autonomousExit() {
    PosUtils.selectTab(0);

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.updateVisionPose(false);
    if (!isFMS) {
      // hopefully plays the music when the robot is enabled (when not connected to an
      // fms)
      // TODO: figure out how to bind this to a button in RobotContainer later
      if (musicTimer == 0) {
        jukebox.play();
      }
      if (musicTimer == 500) {
        jukebox.pause();
      }

      musicTimer++;
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
