// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value; // TODO: what is this!!!???

import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UppiesConstants;
import frc.utilities.PosUtils;

public class UppiesSystem extends SubsystemBase {
  private static SparkMax motorLeft = new SparkMax(UppiesConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static SparkMax motorRight = new SparkMax(UppiesConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private static SparkMax motorLock = new SparkMax(UppiesConstants.LOCK_MOTOR_ID, MotorType.kBrushless);
  private static DigitalInput lockSensor = new DigitalInput(UppiesConstants.LOCK_SENSOR_ID);
  private static DigitalInput cageSensor = new DigitalInput(UppiesConstants.CAGE_SENSOR_ID);

  /** Creates a new UppiesSystem. */
  public UppiesSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Left Pos", this::getPosLeft)
        .withWidget(BuiltInWidgets.kTextView).withPosition(0, 3);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Right Pos", this::getPosRight)
        .withWidget(BuiltInWidgets.kTextView).withPosition(0, 4);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Left Pos", this::getScaledPosLeft)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "center", 0, "max", 1))
        .withPosition(1, 3).withSize(2, 1);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Right Pos", this::getScaledPosRight)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "center", 0, "max", 1))
        .withPosition(1, 4).withSize(2, 1);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Lock Motor Current", this::getLockMotorCurrent)
        .withWidget(BuiltInWidgets.kGraph).withPosition(12, 0);
    Shuffleboard.getTab("Sensor values").addBoolean("Lock Sensor", this::cageLocked)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 4);
    Shuffleboard.getTab("Sensor values").addBoolean("Cage Sensor", this::hasCage)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 3);
  }

  /**
   * Runs the left motor at a set velocity.
   * <p>
   * <b>WARNING: this ignores limits entirely and can damage the robot, consider
   * using another method.</b>
   * </p>
   * 
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runLeftMotor(double velocity) {
    motorLeft.set(velocity);
  }

  /**
   * Runs the right motor at a set velocity.
   * <p>
   * <b>WARNING: this ignores limits entirely and can damage the robot, consider
   * using another method.</b>
   * </p>
   * 
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runRightMotor(double velocity) {
    motorRight.set(velocity);
  }

  /**
   * Tells the Uppies system to run the motor at a set velocity. Properly follows
   * position limits defined by
   * {@link frc.robot.Constants.UppiesConstants#LIMIT_MARGIN},
   * such that the motors are only not allowed to move past a scaled position of
   * {@value frc.robot.Constants.ElevatorConstants#LIMIT_MARGIN} for the bottom
   * limit, or
   * 1.0-{@value frc.robot.Constants.ElevatorConstants#LIMIT_MARGIN} for the top
   * limit.
   * 
   * @param speed The speed to move the motor at, between -1.0 to 1.0 (ie, 1.0
   *              moves the Uppies motors with the intent to [TODO check if 1.0
   *              moves the robot up or down wrt climbing])
   * @see frc.robot.subsystems.UppiesSystem#getScaledPosLeft()
   */
  public void runWithLimits(double speed) {
    runLeftMotor(PosUtils.runWithLimits(speed * UppiesConstants.MOTOR_DESYNC_RATIO, getScaledPosLeft(),
        UppiesConstants.LIMIT_MARGIN));
    runRightMotor(PosUtils.runWithLimits(speed, getScaledPosRight(),
        UppiesConstants.LIMIT_MARGIN));
  }

  /**
   * Gets the position of the left Uppies motor absolute encoder.
   * 
   * @return - the encoder value, in rotations (from 0.0 to 1.0)
   */
  public double getPosLeft() {
    return motorLeft.getAbsoluteEncoder().getPosition();
  }

  /**
   * Gets the position of the right Uppies motor absolute encoder.
   * 
   * @return - the encoder value, in rotations (from 0.0 to 1.0)
   */
  public double getPosRight() {
    return motorRight.getAbsoluteEncoder().getPosition();
  }

  /**
   * Gets the position of the right Uppies motor absolute encoder, after scaling
   * to the possible range of motion.
   * 
   * @return - the encoder value, in percentage of travel distance (from 0.0 to
   *         1.0)
   */
  public double getScaledPosLeft() {
    return PosUtils.mapRange(getPosLeft(), UppiesConstants.LEFT_MIN_POSITION, UppiesConstants.LEFT_MAX_POSITION, 0.0,
        1.0);
  }

  /**
   * Gets the position of the left Uppies motor absolute encoder, after scaling to
   * the possible range of motion.
   * 
   * @return - the encoder value, in percentage of travel distance (from 0.0 to
   *         1.0)
   */
  public double getScaledPosRight() {
    return PosUtils.mapRange(getPosRight(), UppiesConstants.RIGHT_MIN_POSITION, UppiesConstants.RIGHT_MAX_POSITION, 0.0,
        1.0);
  }

  public boolean atLowPosition(double position) {
    boolean reachedRight = false;
    boolean reachedLeft = false;
    if ((getScaledPosRight() - UppiesConstants.LIMIT_MARGIN) <= position) {
      reachedRight = true;
    }

    if ((getScaledPosLeft() - UppiesConstants.LIMIT_MARGIN) <= position) {
      reachedLeft = true;
    }

    return reachedRight && reachedLeft;

  }

  public void runLockMotor(double velocity) {
    motorLock.set(velocity);
  }

  public double getLockMotorCurrent() {
    return motorLock.getOutputCurrent();
  }

  public boolean cageLocked() {
    return lockSensor.get();
  }

  public boolean hasCage() {
    return !cageSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
