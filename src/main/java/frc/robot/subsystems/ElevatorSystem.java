// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utilities.PosUtils;

public class ElevatorSystem extends SubsystemBase implements PosUtils {

  private static SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private static AnalogInput pot = new AnalogInput(ElevatorConstants.ELEVATOR_POT_ID);
  private static DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.ELEVATOR_BOTTOM_LIMIT_ID);
  private static DigitalInput topLimit = new DigitalInput(ElevatorConstants.ELEVATOR_TOP_LIMIT_ID);
  private static PIDController PID = ElevatorConstants.ELEVATOR_PID;

  /**
   * Creates a new elevator system with the values provided in Constants.java. Consists of one motor, a potentiometer, and 2 boolean sensors for the top and bottom limits
   * @see frc.robot.Constants.ElevatorConstants
   */
  public ElevatorSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Elevator Pot", this::getPos);
    Shuffleboard.getTab("Sensor values").addBoolean("Elevator Bottom Limit Switch", this::atBottom);
    Shuffleboard.getTab("Sensor values").addBoolean("Elevator Top Limit Switch", this::atTop);
  }

  /**
   * Runs the Elevator motor at a set speed percentage 
   * <p><b>WARNING: this ignores limit switches entirely, consider using another method</b></p>
   * @param speed The speed percentage to run the motor at (IE, values from -1.0 to 1.0)
   */
  public void runMotor(double velocity) {
    motor.set(velocity);
  }

  /**
   * Returns the velocity of the Elevator motor
   * @return The current velocity of the elevator motor, in Rotations per Minute
   */
  public double getMotorVelocity() {
    return motor.getEncoder().getVelocity();
  }

  /**
   * Gets the current position of the elevator
   * @return The current position of the elevator, in raw potentiometer values.
   */
  public double getPos() {
    return pot.getValue();
  }
  /**
   * checks if the elevator is at the specified limit switch
   * @param top Whether we are checking the top limit switch. If false, will check the bottom limit switch
   * @return True if the elevator is at the limit switch, otherwise false
   * @see #atTop()
   * @see #atBottom()
   */
  public boolean atLimit(boolean top) {
    return top? atTop() : atBottom();
  }

  /**
   * Checks if the elevator is at the bottom limit switch
   * @return whether or not the bottom limit switch is depressed
   * @see #atLimit(boolean)
   */
  public boolean atBottom() {
    return bottomLimit.get();
  }

  /**
   * Checks if the elevator is at the top limit switch
   * @return whether or not the top limit switch is depressed
   * @see #atLimit(boolean)
   */
  public boolean atTop() {
    return topLimit.get();
  }

  /**
   * uses PID to find the speed to move the elevator at to get to the desired position
   * @param desiredPos the desired position, currently in raw potentiometer units
   * @return the speed to move the elevator
   */
  public double getNewSpeed(double desiredPos) {
    return PID.calculate(getPos(), desiredPos);
  }

  /**
   * Checks if the motor position is within tolerance of the desired position (currently "{@value frc.robot.Constants.ElevatorConstants#ELEVATOR_POS_TOLERANCE}", 
   * as set in {@link frc.robot.Constants.ElevatorConstants#ELEVATOR_POS_TOLERANCE}), and if the motor is moving towards the desired position
   * 
   * @param desiredPos the desired position of the joint, in Rotations
   * @return if the motor is within tolerance
   * @see frc.utilities.PosUtils#isOscillating(double, double, double, double, double)
   */
  public boolean isOscillating(double desiredPos) {
    return PosUtils.isOscillating(desiredPos, getPos(), ElevatorConstants.ELEVATOR_POS_TOLERANCE, getMotorVelocity(), ElevatorConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE);
  }

  /**
   * dangerously moves the elevator based on joystick input
   * <p><b>WARNING: Ignores limit switches</b></p>
   * @param joystickInput the input from the joystick
   * @return A command to run the elevator motor
   */
  public Command elevatorManualControl(Double joystickInput) {
    System.out.println("asdfsadf");
    return runEnd(() -> {runMotor(joystickInput);}, () -> {runMotor(0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
