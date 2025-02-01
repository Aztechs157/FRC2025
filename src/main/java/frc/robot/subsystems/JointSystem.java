// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.utilities.PosUtils;

public class JointSystem extends SubsystemBase implements PosUtils {
  private static SparkBase motor;
  private static PIDController PID;

  /** Creates a new JointSystem. */
  public JointSystem(boolean isElbow) {
    motor = isElbow ? new SparkFlex(ElbowConstants.ELBOW_MOTOR_ID, MotorType.kBrushless) : new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    PID = isElbow ? ElbowConstants.ELBOW_PID : WristConstants.WRIST_PID;
  }

  public void runMotor(double velocity) {
    motor.set(velocity);
  }

  private double getMotorVelocity() {
    return motor.getEncoder().getVelocity();
  }

  private double getPos() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  public boolean isOscillating(double desiredPos) {
    return PosUtils.isOscillating(desiredPos, getPos(), ElevatorConstants.ELEVATOR_POS_TOLERANCE, getMotorVelocity(), ElevatorConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE);
  }


  public double getNewSpeed(double desiredPos) {
    return PID.calculate(getPos(), desiredPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
