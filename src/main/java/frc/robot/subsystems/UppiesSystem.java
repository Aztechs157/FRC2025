// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UppiesConstants;
import frc.utilities.PosUtils;

public class UppiesSystem extends SubsystemBase {
  private static SparkMax motorLeft = new SparkMax(UppiesConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static SparkMax motorRight = new SparkMax(UppiesConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new uppiesSystem. */
  public UppiesSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Left Pos", this::getPosLeft);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Right Pos", this::getPosRight);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Left Pos", this::getScaledPosLeft);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Right Pos", this::getScaledPosRight);
  }

  /**
   * Runs the left motor at a set velocity 
   * <p><b>WARNING: this ignores limits entirely and can damage the robot, consider using another method</b></p>
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runLeftMotor(double velocity) {
    motorLeft.set(velocity);
  }

  /**
   * Runs the right motor at a set velocity 
   * <p><b>WARNING: this ignores limits entirely and can damage the robot, consider using another method</b></p>
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runRightMotor(double velocity) {
    motorRight.set(velocity);
  }

  /**
   * tells the Uppies system to run the motor at a set velocity. Properly follows position limits defined by {@link frc.robot.Constants.UppiesConstants#LIMIT_MARGIN}, 
   * such that the motors are only not allowed to move past a scaled position of  {@value frc.robot.Constants.ElevatorConstants#LIMIT_MARGIN} for the bottom limit, or 
   * 1.0-{@value frc.robot.Constants.ElevatorConstants#LIMIT_MARGIN} for the top limit
   * @param speed The speed to move the motor at, between -1.0 to 1.0 (ie, 1.0 moves the uppies motors with the intent to [TODO check if 1.0 moves the robot up or down wrt climbing])
   * @see frc.robot.subsystems.UppiesSystem#getScaledPosLeft()
   */
  public void runWithLimits(double speed) { // TODO: update this to use PID then generalize it in PosUtils
    if (getScaledPosLeft() >= 1.0 - UppiesConstants.LIMIT_MARGIN && speed < 0) {
      runLeftMotor(0);
    } else if (getScaledPosLeft() >= 0.9 - UppiesConstants.LIMIT_MARGIN && speed < 0) {
      runLeftMotor(speed * 0.75);
    } else if (getScaledPosLeft() <= 0.0 + UppiesConstants.LIMIT_MARGIN && speed > 0) {
      runLeftMotor(0);
    } else if (getScaledPosLeft() <= 0.1 + UppiesConstants.LIMIT_MARGIN && speed > 0) {
      runLeftMotor(speed * 0.75);
    } else {
      runLeftMotor(speed);
    }

    if (getScaledPosRight() >= 1.0 - UppiesConstants.LIMIT_MARGIN && speed < 0) {
      runRightMotor(0);
    } else if (getScaledPosRight() >= 0.9 - UppiesConstants.LIMIT_MARGIN && speed < 0) {
      runRightMotor(-speed * 0.75);
    } else if (getScaledPosRight() <= 0.0 + UppiesConstants.LIMIT_MARGIN && speed > 0) {
      runRightMotor(0);
    } else if (getScaledPosRight() <= 0.1 + UppiesConstants.LIMIT_MARGIN && speed > 0) {
      runRightMotor(-speed * 0.75);
    } else {
      runRightMotor(-speed);
    }
  }
  
  /**
   * Gets the position of the left uppies motor absolute encoder
   * @return - the encoder value, in rotations (from 0.0 to 1.0)
   */
  private double getPosLeft() {
    return motorLeft.getAbsoluteEncoder().getPosition();
  }

  /**
   * Gets the position of the right uppies motor absolute encoder
   * @return - the encoder value, in rotations (from 0.0 to 1.0)
   */
  private double getPosRight() {
    return motorRight.getAbsoluteEncoder().getPosition();
  }

  /**
   * Gets the position of the right uppies motor absolute encoder, after scaling to the possible range of motion
   * @return - the encoder value, in percentage of travel distance (from 0.0 to 1.0)
   */
  public double getScaledPosLeft(){
    return PosUtils.mapRange(getPosLeft(), UppiesConstants.LEFT_MIN_POSITION, UppiesConstants.LEFT_MAX_POSITION, 0.0, 1.0);
  }

/**
   * Gets the position of the left uppies motor absolute encoder, after scaling to the possible range of motion
   * @return - the encoder value, in percentage of travel distance (from 0.0 to 1.0)
   */
  public double getScaledPosRight(){
    return PosUtils.mapRange(getPosRight(), UppiesConstants.RIGHT_MIN_POSITION, UppiesConstants.RIGHT_MAX_POSITION, 0.0, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
