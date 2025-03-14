// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.VisionSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private DriveSystem driveSystem;
  private VisionSystem visionSystem;
  private Command driveCommand;

  /** Creates a new DriveToPose. */
  public DriveToPose(DriveSystem driveSystem, VisionSystem visionSystem) {
    this.driveSystem = driveSystem;
    this.visionSystem = visionSystem;
    addRequirements(driveSystem, visionSystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (visionSystem.desiredPose != null) {
      driveCommand = driveSystem.driveToPose(visionSystem.desiredPose);
      driveCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      driveCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return visionSystem.desiredPose == null || driveCommand.isScheduled();
  }
}
