// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.subsystems.ElbowSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.WristSystem;
import frc.utilities.PosUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EnsureSafety extends Command {

  private ElevatorSystem elevator;
  private ElbowSystem elbow;
  private WristSystem wrist;
  private boolean doAnything = false;
  private PositionDetails positionDetails;
  private double position;

  /** Creates a new EnsureSafety. */
  public EnsureSafety(ElevatorSystem elevator, ElbowSystem elbow, WristSystem wrist, PositionDetails positionDetails) {
    this.elevator = elevator;
    this.elbow = elbow;
    this.wrist = wrist;
    this.positionDetails = positionDetails;
    addRequirements(elevator, elbow, wrist);
    // Use addRequirements() here to declare subsystem dependencies.
    position = positionDetails.getElbowPosAtStage(Position.STAGE2.stageNum) - 0.15;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbow.reset();

    if (PosUtils.isWithin(elevator.getScaledPos(), 0, 0.1) && PosUtils.isWithin(elbow.getScaledPos(), 1, 0.1)
        && PosUtils.isWithin(wrist.getScaledPos(), wrist.startingPos, 0.15)) {
      doAnything = true;
    } else {
      doAnything = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (doAnything) {
      elbow.runMotor(elbow.getNewSpeed(position));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elbow.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !doAnything || PosUtils.isWithin(elbow.getScaledPos(), position, 0.1);
  }
}
