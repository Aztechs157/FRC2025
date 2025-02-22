// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.uppies_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.UppiesSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnstallLockies extends InstantCommand {
  private final UppiesSystem uppiesSystem;

  public UnstallLockies(final UppiesSystem uppiesSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.uppiesSystem = uppiesSystem;
    addRequirements(uppiesSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    uppiesSystem.runLockMotor(0);
  }
}
