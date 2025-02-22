// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.uppies_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UppiesSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UppiesLevellingTest extends Command {
  private final UppiesSystem uppiesSystem;
  private final Timer timer = new Timer();
  private double startPosLeft;
  private double startPosRight;

  /** Creates a new TempCommand. */
  public UppiesLevellingTest(final UppiesSystem uppiesSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uppiesSystem);

    this.uppiesSystem = uppiesSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    startPosLeft = uppiesSystem.getPosLeft();
    startPosRight = uppiesSystem.getPosRight();

    uppiesSystem.runRightMotor(0.5);
    uppiesSystem.runLeftMotor(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    uppiesSystem.runWithLimits(0);

    System.out.println("Left Delta: " + (uppiesSystem.getPosLeft() - startPosLeft));
    System.out.println("Right Delta: " + (startPosRight - uppiesSystem.getPosRight()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.25);
  }
}
