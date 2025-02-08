// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.utilities.PosUtils;

public class WristSystem extends SubsystemBase implements PosUtils {
  private static SparkMax motor;
  private static PIDController PID;

  /** Creates a new JointSystem. */

  // TODO: Split into two subsystems because it doesn't like defining different motor types
  // This should be fine if you change the variables to not be static - Katie
  
  /**
   * Creates a new Joint subsystem, meant for the elbow and wrist as they should have very similar functionality.
   * @param isElbow whether or not this joint is the elbow, primarily for choosing the correct PID and displaying the proper name on shuffleboard
   */
  public WristSystem() {
    motor = new SparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless);
    PID = WristConstants.PID;
    Shuffleboard.getTab("Sensor values").addDouble("Wrist Encoder", this::getPos);
    Shuffleboard.getTab("Sensor values").addDouble("Scaled Wrist Encoder", this::getScaledPos);
  }

  /**
   * Runs the motor at a set velocity 
   * <p><b>WARNING: this ignores limits entirely and can damage the robot, consider using another method</b></p>
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runMotor(double velocity) {
    motor.set(velocity);
  }

  /**
   * Get the current speed of the motor
   * @return The speed of the motor, in Rotations per Minute
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

  public double getScaledPos(){
    return PosUtils.mapRange(getPos(), WristConstants.MIN_POSITION, WristConstants.MAX_POSITION, 0.0, 1.0);
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
    return PosUtils.isOscillating(desiredPos, getPos(), WristConstants.POS_TOLERANCE, getMotorVelocity(), WristConstants.MOTOR_VELOCITY_TOLERANCE);
  }

  /**
   * uses PID to find the speed to move the joint at to get to the desired position
   * @param desiredPos the desired position, currently in raw potentiometer units
   * @return the speed to move the joint
   */
  public double getNewSpeed(double desiredPos) {
    return PID.calculate(getPos(), desiredPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}