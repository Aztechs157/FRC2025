// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.elbow_commands.elbowManualControl;
import frc.robot.commands.elevator_commands.elevatorDown;
import frc.robot.commands.elevator_commands.elevatorManualControl;
import frc.robot.commands.intake_commands.EjectCoral;
import frc.robot.commands.intake_commands.IntakeCoral;
import frc.robot.commands.uppies_commands.uppiesManualControl;
import frc.robot.commands.wrist_commands.wristManualControl;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.elbowSystem;
import frc.robot.subsystems.wristSystem;
// import frc.robot.subsystems.VisionSystem;
import frc.robot.subsystems.UppiesSystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final DriveSystem drivetrain = TunerConstants.createDrivetrain();
    private final UppiesSystem uppies = new UppiesSystem();
    private final ElevatorSystem elevator = new ElevatorSystem();
    private final IntakeSystem intake = new IntakeSystem();
    private final elbowSystem elbow = new elbowSystem();
    private final wristSystem wrist = new wristSystem();

    public Command uppiesUpCommand() {
        return new uppiesManualControl(uppies, 0.5);
    }

    public Command uppiesDownCommand() {
        return new uppiesManualControl(uppies, -0.5);
    }

    public Command elevatorStallCommand() {
        return new elevatorManualControl(elevator, 0.057);
    }

    public Command elevatorUpCommand() {
        return new elevatorManualControl(elevator, 0.25);
    }

    public Command elevatorDownCommand() {
        return new elevatorManualControl(elevator, -0.157);
    }

    public Command elbowUpCommand() {
        return new elbowManualControl(elbow, 0.25);
    }

    public Command elbowDownCommand() {
        return new elbowManualControl(elbow, -0.25);
    }

    public Command wristUpCommand() {
        return new wristManualControl(wrist, 0.25);
    }

    public Command wristDownCommand() {
        return new wristManualControl(wrist, -0.25);
    }

    public Command intakeCommand() {
        return new IntakeCoral(intake);
    }

    public Command ejectCommand() {
        return new EjectCoral(intake);
    }

    // public final VisionSystem visionSystem = new VisionSystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 

        driverController.povUp().whileTrue(uppiesUpCommand());
        driverController.povDown().whileTrue(uppiesDownCommand());

        operatorController.povUp().and(operatorController.start()).toggleOnTrue(elevatorStallCommand());
        operatorController.povUp().whileTrue(elevatorUpCommand());
        operatorController.povDown().whileTrue(elevatorDownCommand());

        operatorController.rightTrigger().whileTrue(elbowUpCommand());
        operatorController.rightBumper().whileTrue(elbowDownCommand());

        operatorController.leftTrigger().whileTrue(wristUpCommand());
        operatorController.leftBumper().whileTrue(wristDownCommand());

        operatorController.a().whileTrue(intakeCommand());
        operatorController.b().whileTrue(ejectCommand());      

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
