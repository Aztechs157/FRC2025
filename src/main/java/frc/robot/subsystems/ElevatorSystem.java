// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utilities.PosUtils;

public class ElevatorSystem extends SubsystemBase implements PosUtils {

  private static SparkMax motor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
  private static AnalogInput pot = new AnalogInput(ElevatorConstants.POT_ID);
  private static DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_ID);
  private static DigitalInput topLimit = new DigitalInput(ElevatorConstants.TOP_LIMIT_ID);
  private static PIDController PID = ElevatorConstants.PID;
  private static SlewRateLimiter slew = new SlewRateLimiter(ElevatorConstants.SLEW_RATE_LIMIT_UP,
      ElevatorConstants.SLEW_RATE_LIMIT_DOWN, 0);
  private GenericEntry shuffleboardFeedforward;

  /**
   * Creates a new elevator system with the values provided in Constants.java.
   * Consists of one motor, a potentiometer, and 2 boolean sensors for the top and
   * bottom limits.
   * 
   * @see frc.robot.Constants.ElevatorConstants
   */
  public ElevatorSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Elevator Pot", this::getPos).withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0);
    Shuffleboard.getTab("Sensor values").addDouble("Scaled Elevator Pot", this::getScaledPos)
        .withWidget(BuiltInWidgets.kTextView).withPosition(1, 0).withSize(2, 1);
    Shuffleboard.getTab("Sensor values").addDouble("Scaled Elevator Pot 2", this::getScaledPos)
        .withWidget(BuiltInWidgets.kGraph).withPosition(9, 3);
    Shuffleboard.getTab("Sensor values").addBoolean("Elevator Bottom Limit Switch", this::atBottom)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0);
    Shuffleboard.getTab("Sensor values").addBoolean("Elevator Top Limit Switch", this::atTop)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 1);
    Shuffleboard.getTab("Sensor values").addDouble("Elevator Motor Velocity", this::getMotorVelocity)
        .withWidget(BuiltInWidgets.kGraph).withPosition(9, 0);
    shuffleboardFeedforward = Shuffleboard.getTab("Sensor values").add("Feedforward Setpoint", 0)
        .withWidget(BuiltInWidgets.kGraph).withPosition(12, 3).getEntry();

    ElevatorConstants.NEW_PID.setTolerance(ElevatorConstants.POS_TOLERANCE, ElevatorConstants.MOTOR_VELOCITY_TOLERANCE);
  }

  /**
   * Runs the Elevator motor at a set speed percentage .
   * <p>
   * <b>WARNING: this ignores limit switches entirely, consider using another
   * method.</b>
   * </p>
   * 
   * @param speed - The speed percentage to run the motor at (IE, values from -1.0
   *              to 1.0)
   */
  public void runMotor(double velocity) {
    motor.set(velocity);
  }

  public void runMotorVolts(double volts) {
    motor.setVoltage(volts);
  }

  public void setClosedLoopGoal(double goal) {
    ElevatorConstants.NEW_PID.setGoal(goal);
  }

  public void runClosedLoop() {
    double feedForwardCalc = ElevatorConstants.FEEDFORWARD.calculate(ElevatorConstants.NEW_PID.getSetpoint().velocity);
    shuffleboardFeedforward.setDouble(feedForwardCalc);
    runMotorVolts(ElevatorConstants.NEW_PID.calculate(getScaledPos()) + feedForwardCalc);
  }

  public double runWithLimits(double speed) {
    if (getScaledPos() >= 1.0 - ElevatorConstants.LIMIT_MARGIN && speed < 0) {
      return 0;
    } else if (getScaledPos() >= 0.9 - ElevatorConstants.LIMIT_MARGIN && speed < 0) {
      return speed * 0.75;
    } else if (getScaledPos() <= 0.0 + ElevatorConstants.LIMIT_MARGIN && speed > 0) {
      return 0;
    } else if (getScaledPos() <= 0.1 + ElevatorConstants.LIMIT_MARGIN && speed > 0) {
      return speed * 0.75;
    } else {
      return speed;
    }
  }

  /**
   * Returns the velocity of the Elevator motor.
   * 
   * @return - The current velocity of the elevator motor, in Rotations per Minute
   */
  public double getMotorVelocity() {
    return motor.getEncoder().getVelocity();
  }

  /**
   * Gets the current position of the elevator.
   * 
   * @return - The current position of the elevator, in raw potentiometer values.
   */
  public double getPos() {
    return pot.getValue();
  }

  /**
   * Returns the current position of the Elevator
   * 
   * @return The current value of the encoder, in percentage of total travel
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION, 0.0, 1.0);
  }

  /**
   * Checks if the elevator is at the specified limit switch.
   * 
   * @param top - Whether we are checking the top limit switch. If false, will
   *            check the bottom limit switch
   * @return - True if the elevator is at the limit switch, otherwise false
   * @see #atTop()
   * @see #atBottom()
   */
  public boolean atLimit(boolean top) {
    return top ? atTop() : atBottom();
  }

  /**
   * Checks if the elevator is at the bottom limit switch.
   * 
   * @return Whether or not the bottom limit switch is depressed
   * @see #atLimit(boolean)
   */
  public boolean atBottom() {
    return !bottomLimit.get();
  }

  /**
   * Checks if the elevator is at the top limit switch.
   * 
   * @return Whether or not the top limit switch is depressed
   * @see #atLimit(boolean)
   */
  public boolean atTop() {
    return !topLimit.get();
  }

  /**
   * Uses PID to find the speed to move the elevator at to get to the desired
   * position.
   * 
   * @param desiredPos - the desired position in scaled potentiometer units
   * @return - the speed to move the elevator
   */
  public double getNewSpeed(double desiredPos) {
    return slew.calculate(PID.calculate(getScaledPos(), desiredPos));
  }

  public void reset() {
    slew.reset(0);
    PID.reset();
  }

  public void reset2() {
    ElevatorConstants.NEW_PID.reset(getScaledPos());
  }

  /**
   * Checks if the motor position is within tolerance of the desired position
   * (currently
   * "{@value frc.robot.Constants.ElevatorConstants#ELEVATOR_POS_TOLERANCE}",
   * as set in
   * {@link frc.robot.Constants.ElevatorConstants#ELEVATOR_POS_TOLERANCE}), and if
   * the motor is moving towards the desired position.
   * 
   * @param desiredPos The desired position of the joint, in Rotations
   * @return If the motor is within tolerance
   * @see frc.utilities.PosUtils#isOscillating(double, double, double, double,
   *      double)
   */
  public boolean isOscillating(double desiredPos) {
    return PosUtils.isOscillating(desiredPos, getScaledPos(), ElevatorConstants.POS_TOLERANCE, getMotorVelocity(),
        ElevatorConstants.MOTOR_VELOCITY_TOLERANCE);
  }

  /**
   * Dangerously moves the elevator based on joystick input.
   * <p>
   * <b>WARNING: Ignores limit switches.</b>
   * </p>
   * 
   * @param joystickInput The input from the joystick
   * @return A command to run the elevator motor
   */
  public Command elevatorManualControl(Double joystickInput) {
    System.out.println("asdfsadf");
    return runEnd(() -> {
      runMotor(joystickInput);
    }, () -> {
      runMotor(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
