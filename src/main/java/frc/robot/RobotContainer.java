// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.UppiesConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.elbow_commands.ElbowGoToPosition;
import frc.robot.commands.elbow_commands.ElbowGoToStage;
import frc.robot.commands.elbow_commands.ElbowManualControl;
import frc.robot.commands.elevator_commands.ElevatorGoToStage;
import frc.robot.commands.elevator_commands.ElevatorManualControl;
import frc.robot.commands.elevator_commands.ElevatorClosedLoopControl;
import frc.robot.commands.elevator_commands.ElevatorGoToExtrema;
import frc.robot.commands.elevator_commands.ElevatorGoToPosition;
import frc.robot.commands.intake_commands.EjectCoral;
import frc.robot.commands.intake_commands.IntakeAlgae;
import frc.robot.commands.intake_commands.IntakeCoral;
import frc.robot.commands.intake_commands.PlaceCoral;
import frc.robot.commands.uppies_commands.Lockies;
import frc.robot.commands.uppies_commands.UnstallLockies;
import frc.robot.commands.uppies_commands.UppiesLevellingTest;
import frc.robot.commands.uppies_commands.UppiesManualControl;
import frc.robot.commands.uppies_commands.UppiesToPosition;
import frc.robot.commands.wrist_commands.WristGoToPosition;
import frc.robot.commands.wrist_commands.WristGoToStage;
import frc.robot.commands.wrist_commands.WristManualControl;
import frc.robot.generated.TunerConstants;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.parsing.PositionDetails.Stage;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ElbowSystem;
import frc.robot.subsystems.WristSystem;
// import frc.robot.subsystems.VisionSystem;
import frc.robot.subsystems.UppiesSystem;
import frc.robot.subsystems.VisionSystem;
import frc.utilities.ButtonBox;
import frc.utilities.ButtonBox.ButtonBoxButtons;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final PositionDetails positionDetails = new PositionDetails();

    private final CommandXboxController driverController = new CommandXboxController(0);
    // private final CommandXboxController operatorController = new
    // CommandXboxController(1);
    private final ButtonBox buttonBox = new ButtonBox(1);

    public final DriveSystem drivetrain = TunerConstants.createDrivetrain();
    private final UppiesSystem uppies = new UppiesSystem();
    private final ElevatorSystem elevator = new ElevatorSystem();
    private final IntakeSystem intake = new IntakeSystem();
    private final ElbowSystem elbow = new ElbowSystem();
    private final WristSystem wrist = new WristSystem();
    private final Field2d desiredField = new Field2d();

    public Command UppiesUpCommand() {
        return new UppiesManualControl(uppies, -UppiesConstants.MANUAL_CONTROL_SPEED);
    }

    public Command UppiesDownCommand() {
        return new UppiesManualControl(uppies, UppiesConstants.MANUAL_CONTROL_SPEED);
    }

    public Command UppiesStallCommand() {
        return new UppiesManualControl(uppies, UppiesConstants.STALL_SPEED);
    }

    public Command UppiesWithLock() {
        return new UnstallLockies(uppies).andThen(new Lockies(uppies))
                .andThen(new UppiesToPosition(uppies, UppiesConstants.MANUAL_CONTROL_SPEED, 0))
                .andThen(new UppiesManualControl(uppies, UppiesConstants.STALL_SPEED));
    }

    public Command UnstallUppies() {
        return new UnstallLockies(uppies);
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

    public Command IntakeCoralCommand() {
        return new IntakeCoral(intake);
    }

    public Command IntakeAlgaeCommand() {
        return new IntakeAlgae(intake);
    }

    public Command EjectCommand() {
        return new EjectCoral(intake);
    }

    public Command PlaceCoralCommand() {
        return new PlaceCoral(intake);
    }

    public Command ResetCoralSubsystemsCommand(Position pos) {
        return new ElbowGoToPosition(elbow, positionDetails, Position.CORALSTATION)
                .andThen(new WristGoToPosition(wrist, positionDetails, Position.CORALSTATION))
                .andThen(new ElevatorClosedLoopControl(elevator, positionDetails, pos));
    }

    public Command GoToPositionCommand(Position pos) {
        return (new ElevatorClosedLoopControl(elevator, positionDetails, pos)
                .andThen(new ElevatorManualControl(elevator, ElevatorConstants.STALL_POWER)))
                .alongWith(new ElbowGoToPosition(elbow, positionDetails, pos))
                .alongWith(new WristGoToPosition(wrist, positionDetails, pos));
    }

    public Command GoToBase() {
        return ResetCoralSubsystemsCommand(Position.BASE);
    }

    public Command GoToStage1() {
        return GoToPositionCommand(Position.STAGE1);
    }

    public Command GoToStage2() {
        return GoToPositionCommand(Position.STAGE2);
    }

    public Command GoToStage3() {
        return GoToPositionCommand(Position.STAGE3);
    }

    public Command GoToStage4() {
        return GoToPositionCommand(Position.STAGE4);
    }

    public Command GoToCoralStationStage() {
        return new ElevatorClosedLoopControl(elevator, positionDetails, Position.CORALSTATION)
                .alongWith(new ElbowGoToPosition(elbow, positionDetails, Position.CORALSTATION))
                .alongWith(new WristGoToPosition(wrist, positionDetails, Position.CORALSTATION));
    }

    public Command GoToAlgaeStageLow() {
        return GoToPositionCommand(Position.ALGAE1);
    }

    public Command GoToAlgaeStageHigh() {
        return GoToPositionCommand(Position.ALGAE2);
    }

    public Command DriveToCoralStationPose() {
        Pose2d coralStation = visionSystem.getTagPose(2).get().toPose2d();
        double offsetDistance = 1;
        Pose2d adjustedPose = new Pose2d(
                coralStation.getX() + offsetDistance, // Apply X offset
                coralStation.getY(), // No Y offset
                coralStation.getRotation() // Maintain the same rotation (adjust if needed)
        );

        desiredField.setRobotPose(adjustedPose);
        return drivetrain.driveToPose(adjustedPose);
    }

    public Command DriveToReefPoseLeft() {
        Pose2d reef = visionSystem.getTagPose(7).get().toPose2d();
        double offsetDistanceX = 1;
        double offsetDistanceY = 1;
        Pose2d adjustedPose = new Pose2d(
                reef.getX() + offsetDistanceX, // Apply X offset
                reef.getY() + offsetDistanceY, // No Y offset
                reef.getRotation() // Maintain the same rotation (adjust if needed)
        );
        desiredField.setRobotPose(adjustedPose);
        return drivetrain.driveToPose(adjustedPose);
    }
    
    public Command exitStartingPosition() {
        return new ElbowGoToPosition(elbow, positionDetails, Position.STAGE2);
    }

    public final VisionSystem visionSystem = new VisionSystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ElevatorStall", ElevatorStallCommand());

        NamedCommands.registerCommand("IntakeCoral", IntakeCoralCommand());
        NamedCommands.registerCommand("Intake_Algae", IntakeAlgaeCommand());
        NamedCommands.registerCommand("PlaceCoral", PlaceCoralCommand());
        NamedCommands.registerCommand("ExitStartingPosition", exitStartingPosition());

        NamedCommands.registerCommand("GoToStage1", GoToStage1());
        NamedCommands.registerCommand("GoToStage2", GoToStage2());
        NamedCommands.registerCommand("GoToStage3", GoToStage3());
        NamedCommands.registerCommand("GoToStage4", GoToStage4());

        NamedCommands.registerCommand("GoToCoralStationStage", GoToCoralStationStage());
        NamedCommands.registerCommand("GoToAlgaeStageLow", GoToAlgaeStageLow());
        NamedCommands.registerCommand("GoToAlgaeStageHigh", GoToAlgaeStageHigh());
   
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        Shuffleboard.getTab("vision").add("Desired Position", desiredField);

 }        

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(),
                                ControllerConstants.LEFT_Y_DEADBAND) * modifySpeed(MaxSpeed)) // Drive forward with
                                                                                              // negative Y
                        // (forward)
                        .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(),
                                ControllerConstants.LEFT_X_DEADBAND) * modifySpeed(MaxSpeed)) // Drive left with
                                                                                              // negative X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(),
                                ControllerConstants.RIGHT_X_DEADBAND) * MaxAngularRate) // Drive counterclockwise with
                                                                                        // negative X (left)
                ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.povLeft().onTrue(UppiesWithLock());
        driverController.povRight().onTrue(UnstallUppies());
        // driverController.povRight().onTrue(new UppiesLevellingTest(uppies));
        driverController.povUp().whileTrue(UppiesUpCommand());
        driverController.povDown().whileTrue(UppiesDownCommand());

        driverController.leftBumper().toggleOnTrue(IntakeCoralCommand());
        driverController.leftTrigger().toggleOnTrue(IntakeAlgaeCommand());
        driverController.rightBumper().toggleOnTrue(PlaceCoralCommand());
        driverController.rightTrigger().whileTrue(EjectCommand());

        buttonBox.buttonBinding(ButtonBoxButtons.C1).onTrue(GoToCoralStationStage());
        buttonBox.buttonBinding(ButtonBoxButtons.AL).onTrue(GoToAlgaeStageLow());
        buttonBox.buttonBinding(ButtonBoxButtons.AH).onTrue(GoToAlgaeStageHigh());
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        driverController.start().and(driverController.a()).onTrue(new InstantCommand(() -> {
            Command driveToReefPose = DriveToReefPoseLeft(); // Create the command to drive to the pose
            driveToReefPose.schedule(); // Schedule the command to be executed
        }));
        // Rotates Drive Pods without actaully moving drive motor, might be useful for
        // testing but not sure of any other practical application
        // driverController.a().whileTrue(drivetrain.applyRequest(() -> point
        // .withModuleDirection(new Rotation2d(-driverController.getLeftY(),
        // -driverController.getLeftX()))));

        buttonBox.buttonBinding(ButtonBoxButtons.U3).toggleOnTrue(ElevatorStallCommand());
        buttonBox.buttonBinding(ButtonBoxButtons.U1).whileTrue(ElevatorUpCommand());
        buttonBox.buttonBinding(ButtonBoxButtons.U2).whileTrue(ElevatorDownCommand());

        buttonBox.buttonBinding(ButtonBoxButtons.R3R, true).whileTrue(ElbowUpCommand());
        buttonBox.buttonBinding(ButtonBoxButtons.R3L, true).whileTrue(ElbowDownCommand());

        buttonBox.buttonBinding(ButtonBoxButtons.R4R, true).whileTrue(WristUpCommand());
        buttonBox.buttonBinding(ButtonBoxButtons.R4L, true).whileTrue(WristDownCommand());

        buttonBox.buttonBinding(ButtonBoxButtons.R1, false).onTrue(GoToStage1());
        buttonBox.buttonBinding(ButtonBoxButtons.R2L, false).onTrue(GoToStage2());
        buttonBox.buttonBinding(ButtonBoxButtons.R3L, false).onTrue(GoToStage3());
        buttonBox.buttonBinding(ButtonBoxButtons.R4L, false).onTrue(GoToStage4());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double modifySpeed(final double speed) {
        final var modifier = 1 - driverController.getRightTriggerAxis() * ControllerConstants.PRECISION_DRIVE_MODIFIER;
        return speed * modifier;
    }

    public void updateVisionPose() {
        var pose = visionSystem.getEstimatedGlobalPose();
        if (pose.isPresent()) {
            double visionTime = visionSystem.getTimeStamp();
            drivetrain.addVisionMeasurement(pose.get().toPose2d(), visionTime);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
