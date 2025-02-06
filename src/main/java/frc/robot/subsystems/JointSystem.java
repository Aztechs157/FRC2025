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
