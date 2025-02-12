// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.uppies_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UppiesSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UppiesManualControl extends Command {

  private final UppiesSystem uppiesSystem;
  private final double commandValue;
  /** Creates a new elbowUp. */
  public UppiesManualControl(final UppiesSystem uppiesSystem, double commandValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uppiesSystem);

    this.uppiesSystem = uppiesSystem;
    this.commandValue = commandValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    uppiesSystem.runWithLimits(commandValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    uppiesSystem.runWithLimits(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
