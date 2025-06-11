// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.UppiesConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.VisionCommands.DriveToPose;
import frc.robot.commands.VisionCommands.SetTargetTag;
import frc.robot.commands.elbow_commands.ElbowGoToPosition;
import frc.robot.commands.elbow_commands.ElbowManualControl;
import frc.robot.commands.elbow_commands.EnsureSafety;
import frc.robot.commands.elevator_commands.ElevatorManualControl;
import frc.robot.commands.elevator_commands.ElevatorClosedLoopControl;
import frc.robot.commands.intake_commands.EjectCoral;
import frc.robot.commands.intake_commands.IntakeAlgae;
import frc.robot.commands.intake_commands.IntakeCoral;
import frc.robot.commands.intake_commands.IntakeCoralSimple;
import frc.robot.commands.intake_commands.PlaceCoral;
import frc.robot.commands.intake_commands.PlaceCoralSimple;
import frc.robot.commands.uppies_commands.UppiesManualControl;
import frc.robot.commands.uppies_commands.UppiesToPosition;
import frc.robot.commands.wrist_commands.WristGoToPosition;
import frc.robot.commands.wrist_commands.WristManualControl;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.BetaTunerConstants;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.ElbowSystem;
import frc.robot.subsystems.WristSystem;
import frc.robot.subsystems.UppiesSystem;
import frc.robot.subsystems.VisionSystem;
import frc.utilities.ButtonBox;
import frc.utilities.ButtonBox.ButtonBoxButtons;

@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
    private DigitalInput isBeta = new DigitalInput(5);
    private boolean isButtonBox = true;
    private double MaxSpeed = BetaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                      // top
    // speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second
                                                                                   // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final PositionDetails positionDetails = new PositionDetails(isBeta.get());

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController;
    private final ButtonBox buttonBox;

    @Logged(name = "LEDs")
    public final static LEDSystem prettyLights = new LEDSystem();

    @Logged(name = "drive")
    public final DriveSystem drivetrain = isBeta.get() ? BetaTunerConstants.createDrivetrain()
            : AlphaTunerConstants.createDrivetrain();

    @Logged(name = "uppies")
    private final UppiesSystem uppies = new UppiesSystem(isBeta.get());

    @Logged(name = "elevator")
    private final ElevatorSystem elevator = new ElevatorSystem(isBeta.get());
    @Logged(name = "intake")
    private final IntakeSystem intake = new IntakeSystem(isBeta.get());
    @Logged(name = "elbow")
    private final ElbowSystem elbow = new ElbowSystem(isBeta.get());
    @Logged(name = "wrist")
    private final WristSystem wrist = new WristSystem(isBeta.get());
    @Logged(name = "vision")
    public final VisionSystem visionSystem = new VisionSystem(prettyLights);

    private Command useAutoPosCommand = new WaitCommand(10).repeatedly();
    @Logged(name = "useAutoPos")
    private Trigger useAutoPos = new Trigger(useAutoPosCommand::isScheduled);

    public Command UppiesUpCommand() {
        return new UppiesManualControl(uppies, UppiesConstants.MANUAL_CONTROL_SPEED);
    }

    public Command UppiesDownCommand() {
        return new UppiesManualControl(uppies, -UppiesConstants.MANUAL_CONTROL_SPEED);
    }

    public Command UppiesStallCommand() {
        return new UppiesManualControl(uppies, UppiesConstants.STALL_SPEED);
    }

    public Command ElevatorStallCommand() {
        return new ElevatorManualControl(elevator,
                isBeta.get() ? ElevatorConstants.BETA_STALL_POWER : ElevatorConstants.ALPHA_STALL_POWER);
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
        return new IntakeCoralSimple(intake);
    }

    public Command IntakeAlgaeCommand() {
        return new IntakeAlgae(intake);
    }

    public Command EjectCommand() {
        return new EjectCoral(intake);
    }

    public Command PlaceCoralCommand() {
        return new PlaceCoralSimple(intake);
    }

    public Command ResetCoralSubsystemsCommand() {
        return new ElbowGoToPosition(elbow, positionDetails, Position.STAGE2);
    }

    public Command GoToPositionCommand(Position pos) {
        return new EnsureSafety(elevator, elbow, wrist, positionDetails)
                .andThen(
                        new ElevatorClosedLoopControl(elevator, positionDetails, pos)
                                .andThen(new ElevatorManualControl(elevator,
                                        isBeta.get() ? ElevatorConstants.BETA_STALL_POWER
                                                : ElevatorConstants.ALPHA_STALL_POWER))
                                .alongWith(new ElbowGoToPosition(elbow, positionDetails, pos))
                                .alongWith(new WristGoToPosition(wrist, positionDetails, pos)));
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

    public Command GoToFloorPickup() {
        return GoToPositionCommand(Position.ALGAEFLOOR);
    }

    public Command GoToProcessor() {
        return GoToPositionCommand(Position.ALGAEPROCESSOR);
    }

    public Command GoToBarge() {
        return GoToPositionCommand(Position.BARGEINIT).onlyWhile(() -> elevator.getScaledPos() < 0.8)
                .andThen(GoToPositionCommand(Position.BARGEFINAL));
    }

    public Command GoToCoralStationStage() {
        return GoToPositionCommand(Position.BARGEINIT).onlyWhile(() -> {
            return elevator.getScaledPos() > 0.8 && wrist.getScaledPos() > 0.36;
        }).andThen(GoToPositionCommand(Position.CORALSTATION));
    }

    public Command GoToAlgaeStageLow() {
        return GoToPositionCommand(Position.ALGAE1);
    }

    public Command GoToAlgaeStageHigh() {
        return GoToPositionCommand(Position.ALGAE2);
    }

    public Command DriveToCoralStationPose() {
        return new SetTargetTag(visionSystem, false, Position.CORALSTATION, positionDetails)
                .andThen(new DriveToPose(drivetrain, visionSystem));

    }

    public Command exitStartingPosition() {
        return new ElbowGoToPosition(elbow, positionDetails, Position.STAGE2);
    }

    public Command DriveToReefPoseRight(Position pos) {
        return new SetTargetTag(visionSystem, false, pos, positionDetails)
                .andThen(new DriveToPose(drivetrain, visionSystem));
    }

    public Command DriveToReefPoseLeft(Position pos) {
        return new SetTargetTag(visionSystem, true, pos, positionDetails)
                .andThen(new DriveToPose(drivetrain, visionSystem));
    }

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        if (isButtonBox) {
            buttonBox = new ButtonBox(1);
            operatorController = null;
        } else {
            operatorController = new CommandXboxController(1);
            buttonBox = null;
        }

        NamedCommands.registerCommand("ElevatorStall", ElevatorStallCommand());

        NamedCommands.registerCommand("IntakeCoral", IntakeCoralCommand());
        NamedCommands.registerCommand("IntakeAlgae", IntakeAlgaeCommand());
        NamedCommands.registerCommand("PlaceCoral", PlaceCoralCommand());
        NamedCommands.registerCommand("ExitStartingPosition", exitStartingPosition());

        NamedCommands.registerCommand("GoToStage1", GoToStage1());
        NamedCommands.registerCommand("GoToStage2", GoToStage2());
        NamedCommands.registerCommand("GoToStage3", GoToStage3());
        NamedCommands.registerCommand("GoToStage4", GoToStage4());
        NamedCommands.registerCommand("GoToBarge", GoToBarge());
        NamedCommands.registerCommand("GoToProcessor", GoToProcessor());

        NamedCommands.registerCommand("GoToCoralStationStage", GoToCoralStationStage());
        NamedCommands.registerCommand("GoToAlgaeStageLow", GoToAlgaeStageLow());
        NamedCommands.registerCommand("GoToAlgaeStageHigh", GoToAlgaeStageHigh());

        NamedCommands.registerCommand("GoToReefLeft", DriveToReefPoseLeft(Position.STAGE2));
        NamedCommands.registerCommand("GoToReefRight", DriveToReefPoseRight(Position.STAGE2));

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        Shuffleboard.getTab("Sensor values").addBoolean("isBeta", isBeta::get);
        Shuffleboard.getTab("Sensor values").addBoolean("useAutoPosition", useAutoPos);

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

        // driverController.povRight().onTrue(new UppiesLevellingTest(uppies));
        driverController.povUp().whileTrue(UppiesUpCommand());
        driverController.povDown().whileTrue(UppiesDownCommand());

        driverController.leftBumper().toggleOnTrue(IntakeCoralCommand());
        driverController.leftTrigger().toggleOnTrue(IntakeAlgaeCommand());
        driverController.rightBumper().toggleOnTrue(PlaceCoralCommand());
        // driverController.rightTrigger().whileTrue(EjectCommand());

        if (isButtonBox) {
            buttonBox.buttonBinding(ButtonBoxButtons.C1).onTrue(GoToCoralStationStage());
            buttonBox.buttonBinding(ButtonBoxButtons.AP).onTrue(GoToAlgaeStageLow());
            buttonBox.buttonBinding(ButtonBoxButtons.AL).onTrue(GoToAlgaeStageHigh());
            buttonBox.buttonBinding(ButtonBoxButtons.AH).onTrue(GoToBarge());
            buttonBox.buttonBinding(ButtonBoxButtons.C5).onTrue(GoToFloorPickup());
            buttonBox.buttonBinding(ButtonBoxButtons.C4).onTrue(GoToProcessor());

        } else {
            driverController.a().onTrue(GoToCoralStationStage());
            driverController.x().onTrue(GoToAlgaeStageLow());
            driverController.y().onTrue(GoToAlgaeStageHigh());
        }
        useAutoPosCommand.schedule();
        driverController.rightStick().and(driverController.leftStick()).toggleOnTrue(useAutoPosCommand);

        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));
        // Rotates Drive Pods without actaully moving drive motor, might be useful for
        // testing but not sure of any other practical application
        // driverController.a().whileTrue(drivetrain.applyRequest(() -> point
        // .withModuleDirection(new Rotation2d(-driverController.getLeftY(),
        // -driverController.getLeftX()))));

        if (isButtonBox) {
            buttonBox.buttonBinding(ButtonBoxButtons.U3).toggleOnTrue(ElevatorStallCommand());
            buttonBox.buttonBinding(ButtonBoxButtons.U1).whileTrue(ElevatorUpCommand());
            buttonBox.buttonBinding(ButtonBoxButtons.U2).whileTrue(ElevatorDownCommand());

            buttonBox.buttonBinding(ButtonBoxButtons.R3R, true).whileTrue(ElbowUpCommand());
            buttonBox.buttonBinding(ButtonBoxButtons.R3L, true).whileTrue(ElbowDownCommand());

            buttonBox.buttonBinding(ButtonBoxButtons.R4R, true).whileTrue(WristUpCommand());
            buttonBox.buttonBinding(ButtonBoxButtons.R4L, true).whileTrue(WristDownCommand());

            buttonBox.buttonBinding(ButtonBoxButtons.R1, false).onTrue(GoToStage1());

            buttonBox.buttonBinding(ButtonBoxButtons.R2L, false).onTrue(GoToStage2());
            buttonBox.buttonBinding(ButtonBoxButtons.R2L, false).and(useAutoPos)
                    .onTrue(DriveToReefPoseLeft(Position.STAGE2));

            buttonBox.buttonBinding(ButtonBoxButtons.R3L, false).onTrue(GoToStage3());
            buttonBox.buttonBinding(ButtonBoxButtons.R3L, false).and(useAutoPos)
                    .onTrue(DriveToReefPoseLeft(Position.STAGE3));

            buttonBox.buttonBinding(ButtonBoxButtons.R4L, false).onTrue(GoToStage4());
            buttonBox.buttonBinding(ButtonBoxButtons.R4L, false).and(useAutoPos)
                    .onTrue(DriveToReefPoseLeft(Position.STAGE4));

            buttonBox.buttonBinding(ButtonBoxButtons.R2R, false).onTrue(GoToStage2());
            buttonBox.buttonBinding(ButtonBoxButtons.R2R, false).and(useAutoPos)
                    .onTrue(DriveToReefPoseRight(Position.STAGE2));

            buttonBox.buttonBinding(ButtonBoxButtons.R3R, false).onTrue(GoToStage3());
            buttonBox.buttonBinding(ButtonBoxButtons.R3R, false).and(useAutoPos)
                    .onTrue(DriveToReefPoseRight(Position.STAGE3));

            buttonBox.buttonBinding(ButtonBoxButtons.R4R, false).onTrue(GoToStage4());
            buttonBox.buttonBinding(ButtonBoxButtons.R4R, false).and(useAutoPos)
                    .onTrue(DriveToReefPoseRight(Position.STAGE4));
        } else {
            operatorController.povUp().and(operatorController.start()).toggleOnTrue(ElevatorStallCommand());
            operatorController.povUp().and(operatorController.start().negate()).whileTrue(ElevatorUpCommand());
            operatorController.povDown().whileTrue(ElevatorDownCommand());

            operatorController.rightTrigger().whileTrue(ElbowDownCommand());
            operatorController.rightBumper().whileTrue(ElbowUpCommand());

            operatorController.leftTrigger().whileTrue(WristDownCommand());
            operatorController.leftBumper().whileTrue(WristUpCommand());

            operatorController.a().onTrue(GoToStage1());
            operatorController.x().onTrue(GoToStage2());
            operatorController.x().and(useAutoPos).onTrue(DriveToReefPoseLeft(Position.STAGE2));
            operatorController.b().onTrue(GoToStage3());
            operatorController.b().and(useAutoPos).onTrue(DriveToReefPoseRight(Position.STAGE2));

            operatorController.y().and(operatorController.back().negate()).onTrue(GoToStage4());
            operatorController.y().and(operatorController.back().negate()).and(useAutoPos)
                    .onTrue(DriveToReefPoseRight(Position.STAGE2));
            operatorController.y().and(operatorController.back().onTrue(GoToBarge()));

        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double modifySpeed(final double speed) {
        final var modifier = 1 - driverController.getRightTriggerAxis() * ControllerConstants.PRECISION_DRIVE_MODIFIER;
        return speed * modifier;
    }

    public void updateVisionPose(boolean reset_pose) {
        var speeds = drivetrain.getStateCopy().Speeds;

        var velX = Math.abs(speeds.vxMetersPerSecond);
        var velY = Math.abs(speeds.vyMetersPerSecond);
        var velAngular = Math.abs(speeds.omegaRadiansPerSecond);

        if (velX <= 1 && velY <= 1 && velAngular <= Math.PI) {
            double visionTime = visionSystem.getTimeStamp();
            if (visionTime != 0 && (visionSystem.hasTag)) {
                drivetrain.addVisionMeasurement(visionSystem.getEstimatedGlobalPose2d(),
                        Utils.fpgaToCurrentTime(visionTime));
                if (reset_pose) {
                    drivetrain.resetPose(visionSystem.getEstimatedGlobalPose2d());
                }
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
