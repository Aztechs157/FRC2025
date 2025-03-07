// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.utilities.PosUtils;

public class ElbowSystem extends SubsystemBase implements PosUtils {
  private SparkFlex motor;
  private static DigitalInput encoderInput = new DigitalInput(ElbowConstants.ENCODER_ID);
  private static DutyCycleEncoder encoder = new DutyCycleEncoder(encoderInput);
  private PIDController PID;
    private boolean isBeta;
  
    /** Creates a new JointSystem. */
  
    /**
     * Creates a new Joint subsystem, meant for the elbow and wrist as they should
     * have very similar functionality.
   * @param b 
     */
    public ElbowSystem(boolean isBeta) {
      this.isBeta = isBeta;
      motor = new SparkFlex(ElbowConstants.MOTOR_ID, MotorType.kBrushless);
    PID = ElbowConstants.PID;
    Shuffleboard.getTab("Sensor values").addDouble("Elbow Encoder", this::getPos).withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1);
    Shuffleboard.getTab("Sensor values").addDouble("Scaled Elbow Encoder", this::getScaledPos)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "center", 0, "max", 1))
        .withPosition(1, 1).withSize(2, 1);
    Shuffleboard.getTab("Sensor values").addDouble("Elbow Motor Velocity", this::getMotorVelocity)
        .withWidget(BuiltInWidgets.kGraph).withPosition(6, 0);
  }

  /**
   * Runs the motor at a set velocity.
   * <p>
   * <b>WARNING: this ignores limits entirely and can damage the robot, consider
   * using another method.</b>
   * </p>
   * 
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runMotor(double velocity) {
    motor.set(velocity);
  }

  /**
   * Get the current speed of the motor.
   * 
   * @return The speed of the motor, in Rotations per Minute
   */
  private double getMotorVelocity() {
    return motor.getEncoder().getVelocity();
  }

  /**
   * Returns the current position of the joint.
   * 
   * @return The current value of the absolute encoder, in Rotations
   */
  private double getPos() {
    return encoder.get();
  }

  /**
   * Returns the current position of the joint.
   * 
   * @return The current value of the encoder, in percentage of total travel
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), ElbowConstants.MIN_POSITION, ElbowConstants.MAX_POSITION, 0.0, 1.0);
  }

  /**
   * Checks if the motor position is within tolerance (as set in Constants) of the
   * desired position, and is moving towards the desired position.
   * 
   * @param desiredPos The desired position of the joint, in Rotations
   * @return If the motor is within tolerance
   * @see frc.robot.Constants
   * @see frc.utilities.PosUtils#isOscillating(double, double, double, double,
   *      double)
   */
  public boolean isOscillating(double desiredPos) {
    System.out.println("Elbow | Goal: " + desiredPos + ", Curt: " + getScaledPos() + ", Vel: " + getMotorVelocity());
    return PosUtils.isOscillating(desiredPos, getScaledPos(), ElbowConstants.POS_TOLERANCE, getMotorVelocity(),
        ElbowConstants.MOTOR_VELOCITY_TOLERANCE);
  }

  /**
   * Uses PID to find the speed to move the joint at to get to the desired
   * position.
   * 
   * @param desiredPos The desired position, currently in raw potentiometer units
   * @return The speed to move the joint
   */
  public double getNewSpeed(double desiredPos) {
    return PID.calculate(getScaledPos(), desiredPos);
  }

  public void reset() {
    PID.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
