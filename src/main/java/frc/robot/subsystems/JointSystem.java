// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.utilities.PosUtils;

public class JointSystem extends SubsystemBase implements PosUtils {
  private static SparkBase motor;
  private static PIDController PID;

  /** Creates a new JointSystem. */

  // TODO: Split into two subsystems because it doesn't like defining different motor types
  // This should be fine if you change the variables to not be static - Katie
  
  /**
   * Creates a new Joint subsystem, meant for the elbow and wrist as they should have very similar functionality.
   * @param isElbow - whether or not this joint is the elbow, primarily for choosing the correct PID and displaying the proper name on shuffleboard
   */
  public JointSystem(boolean isElbow) {
    if(isElbow){
      motor = new SparkFlex(ElbowConstants.ELBOW_MOTOR_ID, MotorType.kBrushless);
    }
    else {
      motor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    }
    PID = isElbow ? ElbowConstants.ELBOW_PID : WristConstants.WRIST_PID;
    Shuffleboard.getTab("Sensor values").addDouble(isElbow ? "Elbow Encoder" : "Wrist Encoder", this::getPos);
  }

  /**
   * Runs the motor at a set velocity 
   * @param velocity - Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runMotor(double velocity) {
    motor.set(velocity);
  }
  /**
   * Get the current speed of the motor
   * @return - The speed of the motor, in Rotations per Minute
   */
  private double getMotorVelocity() {
    return motor.getEncoder().getVelocity();
  }

  /**
   * Returns the current position of the joint.
   * @return The current value of the absolute encoder, in Rotations 
   */
  private double getPos() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  /**
   * Checks if the motor position is within tolerance (as set in Constants) of the desired position, and is moving towards the desired position
   * 
   * @param desiredPos the desired position of the joint, in Rotations
   * @return if the motor is within tolerance
   * @see frc.robot.Constants
   * @see frc.utilities.PosUtils#isOscillating(double, double, double, double, double)
   */
  public boolean isOscillating(double desiredPos) {
    return PosUtils.isOscillating(desiredPos, getPos(), ElevatorConstants.ELEVATOR_POS_TOLERANCE, getMotorVelocity(), ElevatorConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE);
  }


  public double getNewSpeed(double desiredPos) {
    return PID.calculate(getPos(), desiredPos);
  }

  public Command elbowManualControl(Double joystickInput) {
    return runEnd(() -> {runMotor(joystickInput);}, () -> {runMotor(0);});
  }
  
  public Command wristManualControl(BooleanSupplier buttonInput, boolean goUp) {
    return runEnd(() -> {if(buttonInput.getAsBoolean()) {runMotor(goUp ? WristConstants.WRIST_MANUAL_CONTROL_SPEED : -WristConstants.WRIST_MANUAL_CONTROL_SPEED);}}, () -> {runMotor(0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
