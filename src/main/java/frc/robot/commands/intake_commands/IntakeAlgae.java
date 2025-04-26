// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  private final IntakeSystem intakeSystem;
  private int holdCounter = 0;

  /** Creates a new IntakeAlgae. */
  public IntakeAlgae(final IntakeSystem intakeSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSystem = intakeSystem;
    addRequirements(intakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holdCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSystem.run(IntakeConstants.INTAKE_ALGAE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      intakeSystem.run(0);
    } else {
      intakeSystem.run(IntakeConstants.ALGAE_HOLDING_SPEED);
      RobotContainer.prettyLights.addMidPattern("Has Algae", 10, LEDPattern.solid(Color.kPurple));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeSystem.getMotorCurrent() > IntakeConstants.ALGAE_HELD_CURRENT) {
      holdCounter++;
    } else {
      holdCounter = 0;
    }
    return holdCounter >= IntakeConstants.ALGAE_HOLD_COUNTER;
  }
}
