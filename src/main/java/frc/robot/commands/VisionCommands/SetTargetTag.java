// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.subsystems.VisionSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTargetTag extends Command {
  private VisionSystem visionSystem;
  private boolean isLeft;
  private Position location;
  private PositionDetails details;

  /** Creates a new SetTargetTag. */
  public SetTargetTag(VisionSystem visionSystem, boolean isLeft, Position location, PositionDetails details) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSystem = visionSystem;
    this.isLeft = isLeft;
    this.location = location;
    this.details = details;
    addRequirements(visionSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double offsetDistanceX = 0;
    double offsetDistanceY = 0;
    var bestTag = visionSystem.findBestTarget();
    if (bestTag != null) {
      int tagID = bestTag.fiducialId;
      Pose2d targetTag = visionSystem.getTagPose(tagID).get().toPose2d();
      switch (location) {
        case ALGAE1, ALGAE2:
          break;
        case BASE:
          break;
        case CORALSTATION:
          break;
        case STAGE1, STAGE2, STAGE3, STAGE4:
          switch (tagID) {
            case 6, 7, 8, 9, 10, 11:
            case 17, 18, 19, 20, 21, 22:
              if (isLeft) {
                offsetDistanceY = details.getLeftOffsetAtStage(location.stageNum);
              } else {
                offsetDistanceY = details.getRightOffsetAtStage(location.stageNum);
              }
              offsetDistanceX = details.getDepthOffsetAtStage(location.stageNum);
              break;
          }
          break;
        default:
          break;

      }

      if (offsetDistanceX != 0) {
        Pose2d adjustedPose = targetTag.plus(new Transform2d(offsetDistanceX, offsetDistanceY, Rotation2d.kPi));
        visionSystem.setDesiredPose(adjustedPose);
      } else {
        visionSystem.setDesiredPose(null);
      }

    } else {
      visionSystem.setDesiredPose(null);
    }

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
