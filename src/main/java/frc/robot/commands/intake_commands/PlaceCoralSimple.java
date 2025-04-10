// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlaceCoralSimple extends Command {
  private final IntakeSystem intakeSystem;
  private Timer timer = new Timer();

  /** Creates a new PlaceCoral. */
  public PlaceCoralSimple(final IntakeSystem intakeSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSystem = intakeSystem;
    addRequirements(intakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSystem.run(-IntakeConstants.PLACE_MOTOR_SPEED);
    timer.start();
    RobotContainer.prettyLights.removeBotPattern("Has Coral");
    RobotContainer.prettyLights.removeBotPattern("Has Algae");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSystem.run(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
