// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private String autoName, newAutoName;
  public final Field2d m_field = new Field2d();

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand().schedule();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateVisionPose();
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {    
    var pose = m_robotContainer.visionSystem.getEstimatedGlobalPose();
    if (pose.isPresent()) {
      m_field.setRobotPose(pose.get().toPose2d());
    }
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if(autoName != newAutoName) {
      autoName = newAutoName;
      if(AutoBuilder.getAllAutoNames().contains(autoName)) {
        try{
        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : pathPlannerPaths) {
          poses.addAll(path.getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
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
  }

  @Override
  public void autonomousPeriodic() {
    var pose = m_robotContainer.visionSystem.getEstimatedGlobalPose();
    if (pose.isPresent()) {
      m_field.setRobotPose(pose.get().toPose2d());
    }
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
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
