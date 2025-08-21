// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.utilities.PosUtils;

@Logged(strategy = Strategy.OPT_OUT)
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
   * 
   * @param b
   */
  public ElbowSystem(boolean isBeta) {
    this.isBeta = isBeta;
    motor = new SparkFlex(ElbowConstants.MOTOR_ID, MotorType.kBrushless);
    PID = ElbowConstants.PID;
    Shuffleboard.getTab("Sensor values").addDouble("Elbow Encoder", this::getPos).withWidget(BuiltInWidgets.kTextView);
    Shuffleboard.getTab("Sensor values").addDouble("Scaled Elbow Encoder", this::getScaledPos)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "center", 0, "max", 1));
    Shuffleboard.getTab("Sensor values").addDouble("Elbow Motor Velocity", this::getMotorVelocity)
        .withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("PID Tuning").add("Elbow PID", ElbowConstants.PID).withWidget(BuiltInWidgets.kPIDController);
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

  public double runWithLimits(double speed) {
    if (getScaledPos() >= 1.0 - ElbowConstants.LIMIT_MARGIN && speed > 0) {
      return 0;
    } else if (getScaledPos() >= 0.9 - ElbowConstants.LIMIT_MARGIN && speed > 0) {
      return speed * 0.75;
    } else if (getScaledPos() <= 0.0 + ElbowConstants.LIMIT_MARGIN && speed < 0) {
      return 0;
    } else if (getScaledPos() <= 0.1 + ElbowConstants.LIMIT_MARGIN && speed < 0) {
      return speed * 0.75;
    } else {
      return speed;
    }
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
    if (isBeta) {
      return PosUtils.mapRange(getPos(), ElbowConstants.BETA_MIN_POSITION, ElbowConstants.BETA_MAX_POSITION, 0.0, 1.0);
    } else {
      return PosUtils.mapRange(getPos(), ElbowConstants.ALPHA_MIN_POSITION, ElbowConstants.ALPHA_MAX_POSITION, 0.0,
          1.0);
    }
  }
  public double getScaledPosAngle() {
    if (isBeta) {
      return PosUtils.mapRange(getPos(), ElbowConstants.BETA_MAX_POSITION, ElbowConstants.BETA_MIN_POSITION, -1.43, 1.0);
    } else {
      return PosUtils.mapRange(getPos(), ElbowConstants.ALPHA_MAX_POSITION, ElbowConstants.ALPHA_MIN_POSITION, -1.43,
          1.0);
    }
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
    SignalLogger.writeDouble("ElbowPosition", getScaledPos());

  }
}
