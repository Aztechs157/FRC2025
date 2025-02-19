// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.uppies_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UppiesConstants;
import frc.robot.subsystems.UppiesSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CageLock extends Command {
  private final UppiesSystem uppiesSystem;
  private final Timer timer = new Timer();
  private boolean cageControlled = false;

  /** Creates a new CageLock. */
  public CageLock(final UppiesSystem uppiesSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.uppiesSystem = uppiesSystem;
    addRequirements(uppiesSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    uppiesSystem.runLockMotor(UppiesConstants.LOCK_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(uppiesSystem.cageLocked()) {
      if (!timer.isRunning()) {
        timer.start();
      } else if (timer.hasElapsed(UppiesConstants.LOCK_TIME)) {
        cageControlled = true;
      }
    } else if (timer.isRunning()) {
      timer.reset();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    uppiesSystem.runLockMotor(UppiesConstants.LOCK_MOTOR_STALL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cageControlled;
  }
}
