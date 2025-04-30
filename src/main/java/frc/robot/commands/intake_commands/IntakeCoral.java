// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private final IntakeSystem intakeSystem;
  private int holdCounter = 0;

  /** Creates a new IntakeCoral. */
  public IntakeCoral(final IntakeSystem intakeSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSystem = intakeSystem;
    addRequirements(intakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holdCounter = 0;
    LEDPattern intakeRunning = LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.25));
    RobotContainer.prettyLights.addMidPattern("Intake Running (Coral)", 11, intakeRunning);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSystem.run(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.prettyLights.removeMidPattern("Intake Running (Coral)");
    if (interrupted) {
      intakeSystem.run(0);
    } else {
      intakeSystem.run(IntakeConstants.CORAL_HOLDING_SPEED);
      RobotContainer.prettyLights.addMidPattern("Has Coral", 11, LEDPattern.solid(Color.kOrange));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeSystem.getMotorCurrent() > IntakeConstants.CORAL_HELD_CURRENT) {
      holdCounter++;
    } else {
      holdCounter = 0;
    }
    return holdCounter >= IntakeConstants.CORAL_HOLD_COUNTER && intakeSystem.hasCoral();
  }
}
