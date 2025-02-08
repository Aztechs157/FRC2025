// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDown extends Command {

  private final ElevatorSystem elevatorSystem;
  private final double commandValue;
  /** Creates a new elevatorDown. */
  public ElevatorDown(final ElevatorSystem elevatorSystem, double commandValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSystem);

    this.elevatorSystem = elevatorSystem;
    this.commandValue = commandValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSystem.runMotor(-commandValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSystem.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
