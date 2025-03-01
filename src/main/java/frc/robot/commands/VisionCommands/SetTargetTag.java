// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTargetTag extends Command {
  private VisionSystem visionSystem;
  private int tagID;

  /** Creates a new SetTargetTag. */
  public SetTargetTag(VisionSystem visionSystem, int tagID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSystem = visionSystem;
    this.tagID = tagID;
    addRequirements(visionSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d reef = visionSystem.getTagPose(tagID).get().toPose2d();
    double offsetDistanceX = 1;
    double offsetDistanceY = 1;
    Pose2d adjustedPose = new Pose2d(
        reef.getX() + offsetDistanceX, // Apply X offset
        reef.getY() + offsetDistanceY, // No Y offset
        reef.getRotation().plus(new Rotation2d(Math.PI)) // Maintain the same rotation (adjust if needed)
    );
    visionSystem.setDesiredPose(adjustedPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
