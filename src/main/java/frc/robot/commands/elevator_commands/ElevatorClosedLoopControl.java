// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.subsystems.ElevatorSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorClosedLoopControl extends Command {

  private final ElevatorSystem elevator;
  private final double position;

  /** Creates a new ElevatorClosedLoopControl. */
  public ElevatorClosedLoopControl(final ElevatorSystem elevator, final PositionDetails positionDetails, Position pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
    position = positionDetails.getElevatorPos(pos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.reset();
    elevator.setClosedLoopGoal(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runClosedLoop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isOscillating(position);
  }
}
