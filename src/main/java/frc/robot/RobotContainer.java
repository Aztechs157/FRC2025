// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.UppiesConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.elbow_commands.ElbowGoToStage;
import frc.robot.commands.elbow_commands.ElbowManualControl;
import frc.robot.commands.elevator_commands.ElevatorGoToStage;
import frc.robot.commands.elevator_commands.ElevatorManualControl;
import frc.robot.commands.elevator_commands.ElevatorGoToExtrema;
import frc.robot.commands.intake_commands.EjectCoral;
import frc.robot.commands.intake_commands.IntakeCoral;
import frc.robot.commands.uppies_commands.UppiesManualControl;
import frc.robot.commands.wrist_commands.WristGoToStage;
import frc.robot.commands.wrist_commands.WristManualControl;
import frc.robot.generated.TunerConstants;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Stage;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ElbowSystem;
import frc.robot.subsystems.WristSystem;
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

    private final PositionDetails positionDetails = new PositionDetails();

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final DriveSystem drivetrain = TunerConstants.createDrivetrain();
    private final UppiesSystem uppies = new UppiesSystem();
    private final ElevatorSystem elevator = new ElevatorSystem();
    private final IntakeSystem intake = new IntakeSystem();
    private final ElbowSystem elbow = new ElbowSystem();
    private final WristSystem wrist = new WristSystem();

    public Command UppiesUpCommand() {
        return new UppiesManualControl(uppies, UppiesConstants.MANUAL_CONTROL_SPEED);
    }

    public Command UppiesDownCommand() {
        return new UppiesManualControl(uppies, -UppiesConstants.MANUAL_CONTROL_SPEED);
    }

    public Command ElevatorStallCommand() {
        return new ElevatorManualControl(elevator, ElevatorConstants.STALL_POWER);
    }

    public Command ElevatorUpCommand() {
        return new ElevatorManualControl(elevator, ElevatorConstants.MANUAL_CONTROL_SPEED_UP);
    }

    public Command ElevatorDownCommand() {
        return new ElevatorManualControl(elevator, -ElevatorConstants.MANUAL_CONTROL_SPEED_DOWN); 
    }

    public Command ElbowUpCommand() {
        return new ElbowManualControl(elbow, ElbowConstants.MANUAL_CONTROL_SPEED);
    }

    public Command ElbowDownCommand() {
        return new ElbowManualControl(elbow, -ElbowConstants.MANUAL_CONTROL_SPEED);
    }

    public Command WristUpCommand() {
        return new WristManualControl(wrist, WristConstants.MANUAL_CONTROL_SPEED);
    }

    public Command WristDownCommand() {
        return new WristManualControl(wrist, -WristConstants.MANUAL_CONTROL_SPEED);
    }

    public Command IntakeCommand() {
        return new IntakeCoral(intake);
    }

    public Command EjectCommand() {
        return new EjectCoral(intake);
    }

     public Command ResetCoralSubsystems() {
         return new ElevatorGoToExtrema(elevator, positionDetails, false);
     }

     public Command GoToStage1() {
         return new ElevatorGoToStage(elevator, positionDetails, 1);
     }

     public Command GoToStage2() {
         return new ElevatorGoToStage(elevator, positionDetails, 2)
         .alongWith(new ElbowGoToStage(elbow, positionDetails, 2))
         .alongWith(new WristGoToStage(wrist, positionDetails, 2))
         .andThen(new ElevatorManualControl(elevator, ElevatorConstants.STALL_POWER));
     }

     public Command GoToStage3() {
         return new ElevatorGoToStage(elevator, positionDetails, 3)
         .alongWith(new ElbowGoToStage(elbow, positionDetails, 3))
         .alongWith(new WristGoToStage(wrist, positionDetails, 3))
         .andThen(new ElevatorManualControl(elevator, ElevatorConstants.STALL_POWER));
     }

     public Command GoToStage4() {
         return new ElevatorGoToStage(elevator, positionDetails, 4);
     }

     //public final VisionSystem visionSystem = new VisionSystem();

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

        driverController.povUp().whileTrue(UppiesUpCommand());
        driverController.povDown().whileTrue(UppiesDownCommand());

        operatorController.povUp().and(operatorController.start()).toggleOnTrue(ElevatorStallCommand());
        operatorController.povUp().whileTrue(ElevatorUpCommand());
        operatorController.povDown().whileTrue(ElevatorDownCommand());

        operatorController.rightTrigger().whileTrue(ElbowUpCommand());
        operatorController.rightBumper().whileTrue(ElbowDownCommand());

        operatorController.leftTrigger().whileTrue(WristUpCommand());
        operatorController.leftBumper().whileTrue(WristDownCommand());

        operatorController.a().whileTrue(IntakeCommand());
        operatorController.b().whileTrue(EjectCommand());   
        
        operatorController.y().onTrue(GoToStage2());
        operatorController.x().onTrue(ResetCoralSubsystems());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
