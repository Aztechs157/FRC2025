// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.subsystems.WristSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristGoToPosition extends Command {
    private final WristSystem wrist;
    private final double position;

    /** Creates a new WristGoToPosition. */
    public WristGoToPosition(final WristSystem wrist, final PositionDetails positionDetails, Position pos) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.wrist = wrist;
        addRequirements(wrist);
        position = positionDetails.getWristPos(pos);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wrist.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wrist.runMotor(wrist.getNewSpeed(position));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wrist.runMotor(0);
        if (interrupted) {
            System.out.println("Wrist interupted");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return wrist.isOscillating(position);
    }
}
