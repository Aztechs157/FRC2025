// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.parsing.PositionDetails;
import frc.robot.subsystems.ElevatorSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToStage extends Command {
  private final ElevatorSystem elevator;
  private final double position;

  /** Creates a new GoToIntermediatePos. */
  public ElevatorGoToStage(final ElevatorSystem elevator, final PositionDetails positionDetails, final int stage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
    position = positionDetails.getElevatorPosAtStage(stage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runMotor(elevator.getNewSpeed(position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isOscillating(position);
  }
}
