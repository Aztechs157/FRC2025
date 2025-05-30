// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.parsing.PositionDetails;
import frc.robot.subsystems.ElbowSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElbowGoToStage extends Command {
  private final ElbowSystem elbow;
  private final double position;

  /** Creates a new ElbowGoToStage. */
  public ElbowGoToStage(final ElbowSystem elbow, final PositionDetails positionDetails, final int stage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elbow = elbow;
    addRequirements(elbow);
    position = positionDetails.getElbowPosAtStage(stage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbow.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbow.runMotor(elbow.getNewSpeed(position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elbow.runMotor(0);
    if (interrupted) {
      System.out.println("Elbow interupted");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elbow.isOscillating(position);
  }
}
