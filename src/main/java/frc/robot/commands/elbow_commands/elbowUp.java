// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JointSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elbowUp extends Command {

  private final JointSystem jointSystem;
  private final double commandValue;
  /** Creates a new elbowUp. */
  public elbowUp(final JointSystem jointSystem, double commandValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(jointSystem);

    this.jointSystem = jointSystem;
    this.commandValue = commandValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    jointSystem.runMotor(commandValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    jointSystem.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
